#include "YourPlanner.h"
#include <rl/plan/SimpleModel.h>
#include <iostream>

#define EXHAUSTED_THRESHOLD 70
//#define PENAL_FACTOR 0.1
YourPlanner::YourPlanner() :
  RrtConConBase()
{
  this->numberCollisions = 0;
}

YourPlanner::~YourPlanner()
{
}

::std::string
YourPlanner::getName() const
{
  return "Your Planner";
}

/*
Improve the nearest function to eliminate exhausted vertex

*/

RrtConConBase::Neighbor
YourPlanner::nearest_nonexh(const Tree &tree, const ::rl::math::Vector &chosen)
{
  //create an empty pair <Vertex, distance> to return
  Neighbor p(Vertex(), (::std::numeric_limits<::rl::math::Real>::max)());
  ::rl::math::Real mod_d_f;

  //Iterate through all vertices to find the nearest neighbour
  for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
  {
    if (tree[*i.first].failure_count > EXHAUSTED_THRESHOLD)
      continue;

    ::rl::math::Real d;
    //::rl::math::Real mod_d = static_cast< ::rl::math::Real >( PENAL_FACTOR*tree[*i.first].failure_count);

    //if(this->numberCollisions > 1000 && this->numberCollisions < 2000) {
      d = this->model->transformedDistance(chosen, *tree[*i.first].q) /*+ mod_d*/;
    /*} else if(this->numberCollisions >= 2000) {
      d = this->model->transformedDistance(chosen, *tree[*i.first].q) + mod_d;
      this->numberCollisions = 0;
    } else {
      d = this->model->transformedDistance(chosen, *tree[*i.first].q);
      mod_d = static_cast< ::rl::math::Real >(0.0);
    }*/

    if (d < p.second)
    {
      p.first = *i.first;
      p.second = d;
      //mod_d_f = mod_d;
    }
  }

  // Compute the square root of distance
  p.second = this->model->inverseOfTransformedDistance(p.second/* - mod_d_f*/);

  return p;
}

/*


*/
void
YourPlanner::choose(::rl::math::Vector& chosen)
{
  RrtConConBase::choose(chosen);
}


/*


*/

RrtConConBase::Vertex 
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  //Do first extend step

  ::rl::math::Real distance = nearest.second;
  ::rl::math::Real step = distance;

  bool reached = false;

  if (step <= this->delta)
  {
    reached = true;
  }
  else
  {
    step = this->delta;
  }

  ::rl::plan::VectorPtr last = ::std::make_shared< ::rl::math::Vector >(this->model->getDof());

  // move "last" along the line q<->chosen by distance "step / distance"
  this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *last);

  this->model->setPosition(*last);
  this->model->updateFrames();

  if (this->model->isColliding())
  {
    //connection fail, counter +1
    tree[nearest.first].failure_count += 1;
    //++this->numberCollisions;
    //std::cout << this->numberCollisions << std::endl;
    return NULL;
  }

  ::rl::math::Vector next(this->model->getDof());

  while (!reached)
  {
    //Do further extend step

    distance = this->model->distance(*last, chosen);
    step = distance;

    if (step <= this->delta)
    {
      reached = true;
    }
    else
    {
      step = this->delta;
    }

    // move "next" along the line last<->chosen by distance "step / distance"
    this->model->interpolate(*last, chosen, step / distance, next);

    this->model->setPosition(next);
    this->model->updateFrames();

    if (this->model->isColliding())
    {
      break;
    }

    *last = next;
  }

  // "last" now points to the vertex where the connect step collided with the environment.
  // Add it to the tree
  Vertex connected = this->addVertex(tree, last);
  this->addEdge(nearest.first, connected, tree);
  //init the counter for connection failure
  tree[connected].failure_count = 0;
  return connected;
  //your modifications here
  //return RrtConConBase::connect(tree, nearest, chosen);
}


/*


*/

RrtConConBase::Vertex 
YourPlanner::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  ::rl::math::Real distance = nearest.second;
  ::rl::math::Real step = (::std::min)(distance, this->delta);

  ::rl::plan::VectorPtr next = ::std::make_shared< ::rl::math::Vector >(this->model->getDof());

  this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *next);

  this->model->setPosition(*next);
  this->model->updateFrames();

  if (!this->model->isColliding())
  {
    Vertex extended = this->addVertex(tree, next);
    this->addEdge(nearest.first, extended, tree);
    return extended;
  }

  return NULL;
  //your modifications here
  //return RrtConConBase::extend(tree, nearest, chosen);
}


/*


*/

bool
YourPlanner::solve()
{
  this->time = ::std::chrono::steady_clock::now();
  // Define the roots of both trees
  this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared< ::rl::math::Vector >(*this->start));
  this->begin[1] = this->addVertex(this->tree[1], ::std::make_shared< ::rl::math::Vector >(*this->goal));

  Tree* a = &this->tree[0];
  Tree* b = &this->tree[1];

  this->tree[0][this->begin[0]].failure_count = 0;
  this->tree[1][this->begin[1]].failure_count = 0;

  ::rl::math::Vector chosen(this->model->getDof());


  while ((::std::chrono::steady_clock::now() - this->time) < this->duration)
  {
    //First grow tree a and then try to connect b.
    //then swap roles: first grow tree b and connect to a.
    for (::std::size_t j = 0; j < 2; ++j)
    {
      //Sample a random configuration
      this->choose(chosen);

      //Find the nearest neighbour in the tree
      Neighbor aNearest = this->nearest_nonexh(*a, chosen);

      //Do a CONNECT step from the nearest neighbour to the sample
      Vertex aConnected = this->connect(*a, aNearest, chosen);

      //If a new node was inserted tree a
      if (NULL != aConnected)
      {
        // Try a CONNECT step form the other tree to the sample
        Neighbor bNearest = this->nearest_nonexh(*b, *(*a)[aConnected].q);
        Vertex bConnected = this->connect(*b, bNearest, *(*a)[aConnected].q);

        if (NULL != bConnected)
        {
          //Test if we could connect both trees with each other
          if (this->areEqual(*(*a)[aConnected].q, *(*b)[bConnected].q))
          {
            this->end[0] = &this->tree[0] == a ? aConnected : bConnected;
            this->end[1] = &this->tree[1] == b ? bConnected : aConnected;
            return true;
          }
        }
      }

      //Swap the roles of a and b
      using ::std::swap;
      swap(a, b);
    }
  }
  //your modifications here
  return false;
}

/*
void YourPlanner::update_tree_cvf(Tree& tree, const Neighbor& nearest)
{
  cons_viol_freq[nearest.first] += 1;
  std::pair<typename Config::in_edge_iterator, typename Config::in_edge_iterator> in_edge_iterator_range = in_edges(nearest.first, tree);
  for(in_edge_iterator i = in_edge_iterator_range.first; i != in_edge_iterator_range.second; i++)
  {
    
  }
  
  
}
*/