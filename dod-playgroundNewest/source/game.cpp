#include "game.h"
#include <vector>
#include <string>
#include <math.h>
#include <assert.h>
#include <cstdint>
#include <emmintrin.h>
#include <immintrin.h>

const int kObjectCount = 100000;
const int kAvoidCount = 32;
const int kTotalObjectCount = kObjectCount + kAvoidCount; 


static float RandomFloat01() { return (float)rand() / (float)RAND_MAX; }
static float RandomFloat(float from, float to) { return RandomFloat01() * (to - from) + from; }


// -------------------------------------------------------------------------------------------------
// components we use in our "game". these are all just simple structs with some data.


// 2D position: just x,y coordinates
struct PositionComponent
{
    float x, y;
};


// Sprite: color, sprite index (in the sprite atlas), and scale for rendering it
struct SpriteComponent
{
    float colorR, colorG, colorB;
    int spriteIndex;
    float scale;
};


// World bounds for our "game" logic: x,y minimum & maximum values
struct WorldBoundsComponent
{
    float xMin, xMax, yMin, yMax;
};

struct PreviousPositionComponent
{
    float xPrev,yPrev;
};

struct AABBComponent
{
    alignas(16) float minX[kTotalObjectCount];
    alignas(16) float minY[kTotalObjectCount];
    alignas(16) float maxX[kTotalObjectCount];
    alignas(16) float maxY[kTotalObjectCount];

};


// Move around with constant velocity. When reached world bounds, reflect back from them.
struct MoveComponent
{
    float velx, vely;

    void Initialize(float minSpeed, float maxSpeed)
    {
        // random angle
        float angle = RandomFloat01() * 3.1415926f * 2;
        // random movement speed between given min & max
        float speed = RandomFloat(minSpeed, maxSpeed);
        // velocity x & y components
        velx = cosf(angle) * speed;
        vely = sinf(angle) * speed;
    }
};


// -------------------------------------------------------------------------------------------------
// super simple "game entities system", using struct-of-arrays data layout.
// we just have an array for each possible component, and a flags array bit bits indicating
// which components are "present".

// "ID" of a game object is just an index into the scene array.
typedef size_t EntityID;

struct Entities
{
    enum
    {
        kFlagPosition = 1<<0,
        kFlagSprite = 1<<1,
        kFlagWorldBounds = 1<<2,
        kFlagMove = 1<<3,
    };

    // arrays of data; the sizes of all of them are the same. EntityID (just an index)
    // is used to access data for any "object/entity". The "object" itself is nothing
    // more than just an index into these arrays.
    
    // names of each object
    std::vector<std::string> m_Names;
    // data for all components
    std::vector<PositionComponent> m_Positions;
    std::vector<SpriteComponent> m_Sprites;
    std::vector<WorldBoundsComponent> m_WorldBounds;
    std::vector<PreviousPositionComponent> m_prevPos;
    std::vector<MoveComponent> m_Moves;
    AABBComponent m_AABB;
    // bit flags for every component, indicating whether this object "has it"
    std::vector<int> m_Flags;
    
    void reserve(size_t n)
    {
        m_Names.reserve(n);
        m_Positions.reserve(n);
        m_Sprites.reserve(n);
        m_WorldBounds.reserve(n);
        m_prevPos.reserve(n);
        m_Moves.reserve(n);
        m_Flags.reserve(n);
    }
    
    EntityID AddEntity(const std::string&& name)
    {
        EntityID id = m_Names.size();
        m_Names.emplace_back(name);
        m_Positions.push_back(PositionComponent());
        m_Sprites.push_back(SpriteComponent());
        m_WorldBounds.push_back(WorldBoundsComponent());
        m_prevPos.push_back(PreviousPositionComponent());
        m_Moves.push_back(MoveComponent());
        m_Flags.push_back(0);
        return id;
    }
};


// The "scene"
static Entities s_Objects;


// -------------------------------------------------------------------------------------------------
// "systems" that we have; they operate on components of game objects


struct MoveSystem
{
    EntityID boundsID; // ID if object with world bounds
    std::vector<EntityID> entities; // IDs of objects that should be moved

    void AddObjectToSystem(EntityID id)
    {
        entities.emplace_back(id);
    }

    void SetBounds(EntityID id)
    {
        boundsID = id;
    }
    
    void UpdateSystem(double time, float deltaTime)
    {
        const WorldBoundsComponent& bounds = s_Objects.m_WorldBounds[boundsID];

        // go through all the objects
        for (size_t io = 0, no = entities.size(); io != no; ++io)
        {
            PositionComponent& pos = s_Objects.m_Positions[io];
            MoveComponent& move = s_Objects.m_Moves[io];
            PreviousPositionComponent& prevPos = s_Objects.m_prevPos[io];


            //set previous position for AABB Collsiion
            prevPos.xPrev = pos.x;
            prevPos.yPrev = pos.y;

            // update position based on movement velocity & delta time
            pos.x += move.velx * deltaTime;
            pos.y += move.vely * deltaTime;
            
            
            // check against world bounds; put back onto bounds and mirror the velocity component to "bounce" back
            if (pos.x < bounds.xMin)
            {
                move.velx = -move.velx;
                pos.x = bounds.xMin;
            }
            if (pos.x > bounds.xMax)
            {
                move.velx = -move.velx;
                pos.x = bounds.xMax;
            }
            if (pos.y < bounds.yMin)
            {
                move.vely = -move.vely;
                pos.y = bounds.yMin;
            }
            if (pos.y > bounds.yMax)
            {
                move.vely = -move.vely;
                pos.y = bounds.yMax;
            }
        }
    }
};

static MoveSystem s_MoveSystem;



// "Avoidance system" works out interactions between objects that "avoid" and "should be avoided".
// Objects that avoid:
// - when they get closer to things that should be avoided than the given distance, they bounce back,
// - also they take sprite color from the object they just bumped into
struct AvoidanceSystem
{
    // things to be avoided: distances to them, and their IDs
    std::vector<float> avoidDistanceList;
    std::vector<EntityID> avoidList;
    
    // objects that avoid: their IDs
    std::vector<EntityID> objectList;

    void AddAvoidThisObjectToSystem(EntityID id, float distance)
    {
        avoidList.emplace_back(id);
        avoidDistanceList.emplace_back(distance * distance);
    }
    
    void AddObjectToSystem(EntityID id)
    {
        objectList.emplace_back(id);
    }
    
    static float DistanceSq(const PositionComponent& a, const PositionComponent& b)
    {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        return dx * dx + dy * dy;
    }

    void FixSpawnColliding(WorldBoundsComponent bounds)
    {
        /*
        for (size_t io = 0, no = objectList.size(); io != no; ++io)
        {
            EntityID go = objectList[io];
            const PositionComponent& myposition = s_Objects.m_Positions[go];
            const PreviousPositionComponent& prevPosition = s_Objects.m_prevPos[go];

            //s_Objects.m_AABB.minX[go] = myposition.x - 0.25;
            //s_Objects.m_AABB.maxX[go] = myposition.x + 0.25;
            //s_Objects.m_AABB.minY[go] = myposition.y - 0.25;
            //s_Objects.m_AABB.maxY[go] = myposition.y + 0.25;
            //float minX = myposition.x - 0.25;
            //float maxX = myposition.x + 0.25;
            //float minY = myposition.y - 0.25;
            //float maxY = myposition.y + 0.25;

            //s_Objects.

            bool collidingWithObstacle = true;
            while (collidingWithObstacle)
            {
                collidingWithObstacle = false;
                // check each thing in avoid list
                for (size_t ia = 0, na = avoidList.size(); ia != na; ++ia)
                {
                    float avDistance = avoidDistanceList[ia];
                    EntityID avoid = avoidList[ia];
                    const PositionComponent& avoidposition = s_Objects.m_Positions[avoid];

                    float avoidMinX = avoidposition.x - 1.0;
                    float avoidMaxX = avoidposition.x + 1.0;
                    float avoidMinY = avoidposition.y - 1.0;
                    float avoidMaxY = avoidposition.y + 1.0;

                    // is our position closer to "thing to avoid" position than the avoid distance?
                    //if (DistanceSq(myposition, avoidposition) < avDistance)
                    
                    if (avoidMaxX > minX&& avoidMinX < maxX && avoidMaxY > minY&& avoidMinY < maxY)
                    {
                        s_Objects.m_Positions[go].x = RandomFloat(bounds.xMin, bounds.xMax);
                        s_Objects.m_Positions[go].y = RandomFloat(bounds.yMin, bounds.yMax);
                        minX = myposition.x - 0.25;
                        maxX = myposition.x + 0.25;
                        minY = myposition.y - 0.25;
                        maxY = myposition.y + 0.25;
                        collidingWithObstacle = true;
                        break;
                    }
                }
            }
        }
        */

        //depreciated SIMD attempt for now
        /*
        for (size_t io = 0, no = objectList.size(); io != no; ++io)
        {
            EntityID go = objectList[io];
            const PositionComponent& myposition = s_Objects.m_Positions[go];
            s_Objects.m_AABB.minX[go] = myposition.x - 0.25;
            s_Objects.m_AABB.maxX[go] = myposition.x + 0.25;
            s_Objects.m_AABB.minY[go] = myposition.y - 0.25;
            s_Objects.m_AABB.maxY[go] = myposition.y + 0.25;
        }
        for (size_t ia = 0, na = avoidList.size(); ia != na; ++ia)
        {
            EntityID avoid = avoidList[ia];
            const PositionComponent& avoidposition = s_Objects.m_Positions[avoid];

            s_Objects.m_AABB.minX[avoid] = avoidposition.x - 1.0;
            s_Objects.m_AABB.maxX[avoid] = avoidposition.x + 1.0;
            s_Objects.m_AABB.minY[avoid] = avoidposition.y - 1.0;
            s_Objects.m_AABB.maxY[avoid] = avoidposition.y + 1.0;
        }
        */

        /*
        __m512 collisionResults = _mm512_set1_ps(1);
        __m512 minXBounds = _mm512_load_ps(s_Objects.m_AABB.minX );
        __m512 maxXBounds = _mm512_load_ps(s_Objects.m_AABB.maxX );
        __m512 minYBounds = _mm512_load_ps(s_Objects.m_AABB.minY );
        __m512 maxYBounds = _mm512_load_ps(s_Objects.m_AABB.maxY );
        __m512 avoidMinXBounds = _mm512_load_ps(s_Objects.m_AABB.minX);
        __m512 avoidMaxXBounds = _mm512_load_ps(s_Objects.m_AABB.maxX);
        __m512 avoidMinYBounds = _mm512_load_ps(s_Objects.m_AABB.minY);
        __m512 avoidMaxYBounds = _mm512_load_ps(s_Objects.m_AABB.maxY);
        for (size_t io = 0, no = objectList.size(); io < no; io +=16)
        {
            minXBounds = _mm512_load_ps(s_Objects.m_AABB.minX + io);
            maxXBounds = _mm512_load_ps(s_Objects.m_AABB.maxX + io);
            minYBounds = _mm512_load_ps(s_Objects.m_AABB.minY + io);
            maxYBounds = _mm512_load_ps(s_Objects.m_AABB.maxY + io);
            for (size_t ia = 0, na = avoidList.size(); ia != na; ++ia)
            {
                EntityID avoid = avoidList[ia];
                avoidMinXBounds = _mm512_load_ps(s_Objects.m_AABB.minX + avoid);
                avoidMaxXBounds = _mm512_load_ps(s_Objects.m_AABB.maxX + avoid);
                avoidMinYBounds = _mm512_load_ps(s_Objects.m_AABB.minY + avoid);
                avoidMaxYBounds = _mm512_load_ps(s_Objects.m_AABB.maxY + avoid);

                //__m512i minMask = _mm512_cmp_ps_mask(minXBounds,avoidMinXBounds)
            }
        }
        */
    }
    
    void ResolveCollision(EntityID id, float deltaTime,EntityID avoidID)
    {
        PositionComponent& pos = s_Objects.m_Positions[id];
        MoveComponent& move = s_Objects.m_Moves[id];
        MoveComponent& avoidMove = s_Objects.m_Moves[avoidID];
        // flip velocity
        move.velx = -move.velx;
        move.vely = -move.vely;
        
        // move us out of collision, by moving just a tiny bit more than we'd normally move during a frame
        pos.x += (move.velx * deltaTime * 1.6f) + (avoidMove.velx * deltaTime);
        pos.y += (move.vely * deltaTime * 1.6f) + (avoidMove.vely * deltaTime);
    }
    
    void UpdateSystem(double time, float deltaTime)
    {
        //old code
        /*
        // go through all the objects
        for (size_t io = 0, no = objectList.size(); io != no; ++io)
        {
            EntityID go = objectList[io];
            const PositionComponent& myposition = s_Objects.m_Positions[go];
            const PreviousPositionComponent& prevPosition = s_Objects.m_prevPos[go];

            float minX = (myposition.x > prevPosition.xPrev) ? prevPosition.xPrev : myposition.x;
            minX -= 0.25f;
            float maxX = (myposition.x > prevPosition.xPrev) ? myposition.x : prevPosition.xPrev;
            maxX += 0.25f;
            float minY = (myposition.y > prevPosition.yPrev) ? prevPosition.yPrev : myposition.y;
            minY -= 0.25f;
            float maxY = (myposition.y > prevPosition.yPrev) ? myposition.y : prevPosition.yPrev;
            maxY += 0.25f;
            // check each thing in avoid list
            for (size_t ia = 0, na = avoidList.size(); ia != na; ++ia)
            {
                float avDistance = avoidDistanceList[ia];
                EntityID avoid = avoidList[ia];
                const PositionComponent& avoidposition = s_Objects.m_Positions[avoid];
                const PreviousPositionComponent& avoidPrevPosition = s_Objects.m_prevPos[avoid];

                float avoidMinX = (avoidposition.x > avoidPrevPosition.xPrev) ? avoidPrevPosition.xPrev : avoidposition.x;
                avoidMinX -= 0.55f;
                float avoidMaxX = (avoidposition.x > avoidPrevPosition.xPrev) ? avoidposition.x : avoidPrevPosition.xPrev;
                avoidMaxX += 0.55f;
                float avoidMinY = (avoidposition.y > avoidPrevPosition.yPrev) ? avoidPrevPosition.yPrev : avoidposition.y;
                avoidMinY -= 0.55f;
                float avoidMaxY = (avoidposition.y > avoidPrevPosition.yPrev) ? avoidposition.y : avoidPrevPosition.yPrev;
                avoidMaxY += 0.55f;

                // is our position closer to "thing to avoid" position than the avoid distance?
                //if (DistanceSq(myposition, avoidposition) < avDistance)
                if (avoidPrevPosition.xPrev != 0 && avoidPrevPosition.yPrev != 0)
                {
                    if (avoidMaxX > minX&& avoidMinX < maxX && avoidMaxY > minY&& avoidMinY < maxY)
                    {
                        ResolveCollision(go, deltaTime,avoid);

                        // also make our sprite take the color of the thing we just bumped into
                        SpriteComponent& avoidSprite = s_Objects.m_Sprites[avoid];
                        SpriteComponent& mySprite = s_Objects.m_Sprites[go];
                        mySprite.colorR = avoidSprite.colorR;
                        mySprite.colorG = avoidSprite.colorG;
                        mySprite.colorB = avoidSprite.colorB;
                    }
                }
            }
            
        }*/

        for (size_t io = 0, no = objectList.size(); io < no; io += 16)
        {
            //__m512 currentX = _mm512_load_ps()
           //_mm512_min_ps
        }
        for (size_t ia = 0, na = avoidList.size(); ia != na; ++ia)
        {
        }
    }

    
};

static AvoidanceSystem s_AvoidanceSystem;


// -------------------------------------------------------------------------------------------------
// "the game"


extern "C" void game_initialize(void)
{
    s_Objects.reserve(1 + kObjectCount + kAvoidCount);
    
    // create "world bounds" object
    WorldBoundsComponent bounds;
    {
        EntityID go = s_Objects.AddEntity("bounds");
        s_Objects.m_WorldBounds[go].xMin = -80.0f;
        s_Objects.m_WorldBounds[go].xMax =  80.0f;
        s_Objects.m_WorldBounds[go].yMin = -50.0f;
        s_Objects.m_WorldBounds[go].yMax =  50.0f;
        bounds = s_Objects.m_WorldBounds[go];
        s_Objects.m_Flags[go] |= Entities::kFlagWorldBounds;
        s_MoveSystem.SetBounds(go);
    }
    
    // create regular objects that move
    for (auto i = 0; i < kObjectCount; ++i)
    {
        EntityID go = s_Objects.AddEntity("object");

        // position it within world bounds
        s_Objects.m_Positions[go].x = RandomFloat(bounds.xMin, bounds.xMax);
        s_Objects.m_Positions[go].y = RandomFloat(bounds.yMin, bounds.yMax);
        s_Objects.m_Flags[go] |= Entities::kFlagPosition;

        
        // setup a sprite for it (random sprite index from first 5), and initial white color
        s_Objects.m_Sprites[go].colorR = 1.0f;
        s_Objects.m_Sprites[go].colorG = 1.0f;
        s_Objects.m_Sprites[go].colorB = 1.0f;
        s_Objects.m_Sprites[go].spriteIndex = rand() % 5;
        s_Objects.m_Sprites[go].scale = 1.0f;
        s_Objects.m_Flags[go] |= Entities::kFlagSprite;

        // make it move
        s_Objects.m_Moves[go].Initialize(0.5f, 0.7f);
        s_Objects.m_Flags[go] |= Entities::kFlagMove;
        s_MoveSystem.AddObjectToSystem(go);

        // make it avoid the bubble things, by adding to the avoidance system
        s_AvoidanceSystem.AddObjectToSystem(go);
    }

    // create objects that should be avoided
    for (auto i = 0; i < kAvoidCount; ++i)
    {
        EntityID go = s_Objects.AddEntity("toavoid");
        
        // position it in small area near center of world bounds
        s_Objects.m_Positions[go].x = RandomFloat(bounds.xMin, bounds.xMax) * 0.2f;
        s_Objects.m_Positions[go].y = RandomFloat(bounds.yMin, bounds.yMax) * 0.2f;
        s_Objects.m_Flags[go] |= Entities::kFlagPosition;

        // setup a sprite for it (6th one), and a random color
        s_Objects.m_Sprites[go].colorR = RandomFloat(0.5f, 1.0f);
        s_Objects.m_Sprites[go].colorG = RandomFloat(0.5f, 1.0f);
        s_Objects.m_Sprites[go].colorB = RandomFloat(0.5f, 1.0f);
        s_Objects.m_Sprites[go].spriteIndex = 5;
        s_Objects.m_Sprites[go].scale = 2.0f;
        s_Objects.m_Flags[go] |= Entities::kFlagSprite;
        
        // make it move, slowly
        s_Objects.m_Moves[go].Initialize(0.1f, 0.2f);
        s_Objects.m_Flags[go] |= Entities::kFlagMove;
        s_MoveSystem.AddObjectToSystem(go);

        // add to avoidance this as "Avoid This" object
        s_AvoidanceSystem.AddAvoidThisObjectToSystem(go, 1.3f);
    }

    //Take care of any objects that spawned on top of avoids
    s_AvoidanceSystem.FixSpawnColliding(bounds);
}


extern "C" void game_destroy(void)
{
}


extern "C" int game_update(sprite_data_t* data, double time, float deltaTime)
{
    int objectCount = 0;
    
    // update object systems
    s_MoveSystem.UpdateSystem(time, deltaTime);
    s_AvoidanceSystem.UpdateSystem(time, deltaTime);

    // go through all objects
    for (size_t i = 0, n = s_Objects.m_Flags.size(); i != n; ++i)
    {
        // For objects that have a Position & Sprite on them: write out
        // their data into destination buffer that will be rendered later on.
        //
        // Using a smaller global scale "zooms out" the rendering, so to speak.
        float globalScale = 0.05f;
        if ((s_Objects.m_Flags[i] & Entities::kFlagPosition) && (s_Objects.m_Flags[i] & Entities::kFlagSprite))
        {
            sprite_data_t& spr = data[objectCount++];
            const PositionComponent& pos = s_Objects.m_Positions[i];
            spr.posX = pos.x * globalScale;
            spr.posY = pos.y * globalScale;
            const SpriteComponent& sprite = s_Objects.m_Sprites[i];
            spr.scale = sprite.scale * globalScale;
            spr.colR = sprite.colorR;
            spr.colG = sprite.colorG;
            spr.colB = sprite.colorB;
            spr.sprite = (float)sprite.spriteIndex;
        }
    }
    return objectCount;
}

