#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D length = end - start;
        // creat rope
        // creat spring
        for(int node_index = 0;node_index < num_nodes;node_index ++)
        {
            masses.push_back(new Mass(length * ((float)node_index / (float)num_nodes),node_mass,false));
            if (node_index != 0)
            {
                springs.push_back(new Spring(masses[node_index - 1],masses[node_index],k));
                /* code */
            }
        }
        // springs[0]->m1->forces = Vector2D(2,2);
        // std::cout <<"springForceBA"<< masses[0]->forces.x <<" "<<masses[0]->forces.y<<std::endl;
        
//        Comment-in this part when you implement the constructor
       for (auto &i : pinned_nodes) {
           masses[i]->pinned = true;
       }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            Vector2D substract = s->m1->position - s->m2->position;
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D springForceBA = s->k * substract.unit() * (substract.norm() - s->rest_length);

            s->m1->forces += -springForceBA;
            s->m2->forces += springForceBA;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                m->forces += gravity;
                m->forces += m->velocity * -0.005;//air bumping factor
                //m->position += m->velocity * delta_t;
                m->velocity += (m->forces / m->mass) * delta_t;
                m->position += m->velocity * delta_t;
                // TODO (Part 2): Add global damping

            }
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                                
                m->position = m->position + (1-0.00005) * (m->position - m->last_position) + gravity/m->mass * delta_t *delta_t;
                
                m->last_position = temp_position;
            }
            m->forces = Vector2D(0, 0);
        }
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            // according to the pinstate of mass
            // adjust the position
            Vector2D substract = s->m1->position - s->m2->position;

            substract = substract.unit() * (substract.norm() - s->rest_length) ;

            if (s->m1->pinned)
            {
                s->m2->position += substract;
            }
            else if(s->m2->pinned)
            {
                s->m1->position -= substract;
            }
            else
            {
                s->m2->position += substract/2.0;
                s->m1->position -= substract/2.0;
            }
        }
    }
}
