#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
    vector<double> widths, heights;
    double deltaW = width / (num_width_points - 1);
    double deltaH = height / (num_height_points - 1);
    bool currPin = false;
    vector<double> pos;
    vector<int> indices;

    uniform_real_distribution<double> unif(-0.001, 0.001); // from https://stackoverflow.com/questions/2704521/generate-random-double-numbers-in-c
    default_random_engine re;
    double vert_z;

    for (int i = 0; i < num_width_points; i++) {
        widths.push_back(i * deltaW);
    }
    for (int j = 0; j < num_height_points; j++) {
        heights.push_back(j * deltaH);
    }

    for (int j = 0; j < num_height_points; j++) {
        for (int i = 0; i < num_width_points; i++) {
            indices.clear();
            indices.push_back(i);
            indices.push_back(j);
            pos.clear();
            pos.push_back(widths[i]);
            pos.push_back(heights[j]);
            currPin = find(pinned.begin(), pinned.end(), indices) != pinned.end();
            if (orientation == HORIZONTAL) {
                point_masses.emplace_back(Vector3D(pos[0], 1, pos[1]), currPin);
            }
            else {
                vert_z = unif(re);
                point_masses.emplace_back(Vector3D(pos[0], pos[1], vert_z), currPin);
            }
        }
    }
    

    for (int i = 0; i < point_masses.size(); i++) {
        int x = i % num_width_points;
        int y = floor(i / num_width_points);
        PointMass* curr = &point_masses[i];

        // structural constraints
        if (x != 0) {
            springs.emplace_back(curr, curr - 1, STRUCTURAL);                           // left
        }
        if (y != 0) {
            springs.emplace_back(curr, curr - num_width_points, STRUCTURAL);            // above
        }

        // shearing constraints
        if (x >= 1 && y >= 1) {
            springs.emplace_back(curr, curr - 1 - num_width_points, SHEARING);          // diagonal upper left
        }
        if (x < num_width_points - 1 && y >= 1) {
            springs.emplace_back(curr, curr - num_width_points + 1, SHEARING);          // diagonal upper right
        }

        // bending constraints
        if (x >= 2) {
            springs.emplace_back(curr, curr - 2, BENDING);                              // two to left
        }
        if (y >= 2) {
            springs.emplace_back(curr, curr - num_width_points * 2, BENDING);           // two above
        }
    }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;
  Vector3D currForce;

  // Clear previous forces
  for (PointMass &p : point_masses) {
      p.forces *= 0;
  }


  // TODO (Part 2): Compute total force acting on each point mass.
  Vector3D totExt;
  for (int j = 0; j < external_accelerations.size(); j++) {
      totExt += external_accelerations[j] * mass;
  }
  for (PointMass& p : point_masses) {
      p.forces += totExt;
  }

  for (int i = 0; i < springs.size(); i++) {
      Spring s = springs[i];
      e_spring_type t = s.spring_type;
      if (t == STRUCTURAL && !cp->enable_structural_constraints) {
          continue;
      }
      else if (t == SHEARING && !cp->enable_shearing_constraints) {
          continue;
      }
      else if (t == BENDING && !cp->enable_bending_constraints) {
          continue;
      }
      Vector3D dist = s.pm_a->position - s.pm_b->position;
      currForce = cp->ks * (dist / dist.norm()) * (dist.norm() - s.rest_length);
      if (t == BENDING) {
          currForce *= 0.2;
      }
      s.pm_a->forces -= currForce;
      s.pm_b->forces += currForce;
  }


  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (PointMass& pm : point_masses) {
      if (pm.pinned) {
          continue;
      }
      Vector3D newPos = pm.position + (1 - (cp->damping / 100)) * (pm.position - pm.last_position) + (pm.forces / mass) * pow(delta_t, 2);
      pm.last_position = pm.position;
      pm.position = newPos;
  }

  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
  for (PointMass& pm : point_masses) {
      self_collide(pm, simulation_steps);
  }

  // TODO (Part 3): Handle collisions with other primitives.
  for (PointMass& pm : point_masses) {
      for (CollisionObject* co : *collision_objects) {
          co->collide(pm);
      }
  }

  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (int i = 0; i < springs.size(); i++) {
      Spring s = springs[i];
      Vector3D stretch_dist = s.pm_a->position - s.pm_b->position;
      if (stretch_dist.norm() > s.rest_length * 1.1) {
          double excess_length = stretch_dist.norm() - s.rest_length * 1.1;
          stretch_dist.normalize();
          stretch_dist *= excess_length;
          if (s.pm_a->pinned) {
              s.pm_b->position += stretch_dist;
          }
          else if (s.pm_b->pinned) {
              s.pm_a->position -= stretch_dist;
          }
          else {
              s.pm_a->position -= stretch_dist * 0.5;
              s.pm_b->position += stretch_dist * 0.5;
          }
      }

  }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (PointMass& pm : point_masses) {
      float key = hash_position(pm.position);
      if (map.find(key) == map.end()) {
          map[key] = new vector<PointMass*>();
      }
      map[key]->push_back(&pm);
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
    vector<PointMass*> candidates = *map[hash_position(pm.position)];
    if (candidates.size() == 1) {
        return;
    }
    Vector3D corr = Vector3D(0, 0, 0);
    int num_vec = 0;
    for (PointMass* c : candidates) {
        Vector3D diff = c->position - pm.position;
        if (diff.norm() == 0) {
            continue;
        }
        else if (diff.norm() <= 2 * thickness) {
            corr -= diff.unit() * (2 * thickness - diff.norm());
            num_vec += 1;
        }
    }
    if (num_vec == 0) {
        return;
    }
    corr /= num_vec;
    pm.position += corr / simulation_steps;
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
    double w = 3 * width / num_width_points;
    double h = 3 * height / num_height_points;
    double t = max(w, h);
    int x = floor(pos.x / w);
    int y = floor(pos.y / h);
    int z = floor(pos.z / t);
  return x * 31*31 + y * 31 + z; 
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
