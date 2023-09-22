# Sample-Driven Connectivity Learning (SDCL) sampler  {#SDCLSampler}

[TOC]

Narrow passage motion planning is hard in general. SDCL as a sampler resolves narrow passage motion planning problems by generating samples that are more likely to exist in configuration space narrow passages. More information can be found [here]<http://www.neil.dantam.name/papers/li2023sdcl.pdf>. 

## Using the SDCL Sampler

SDCL sampler in OMPL requires two additional packages to build, [ThunderSVM]<https://github.com/Xtra-Computing/thundersvm.git> for training SVM on GPU and [NLopt]<https://nlopt.readthedocs.io/en/latest/> for solving non-linear optimization problems. CMake needs to be able to find these two packages for the SDCL sampler to build. Once the SDCL sampler is built, we can call it similar to the examples [here]<https://ompl.kavrakilab.org/samplers.html>. SDCL requires both a planner and a space to construct. The following shows how to use it with lambda function. 

```
namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std::placeholders;  
ob::ValidStateSamplerPtr allocSDCLValidStateSampler(const ob::SpaceInformation *si, const ob::PlannerPtr planner)
{
    auto sampler(std::make_shared<ob::SDCLValidStateSampler>(si, planner));
    return sampler;
}
```  
Then, to call this sampler allocator after the planner setup step, 
```
auto allocSDCLValidStateSampler_partial = [&](const ob::SpaceInformation *si){ return allocSDCLValidStateSampler(si, planner); };
si->setValidStateSamplerAllocator(allocSDCLValidStateSampler_partial);
```

## Limitations

### Limited to Real Vector Space
SDCL sampler only works with `ompl::base::RealVectorStateSpace`, because the samples are used internally to train a Euclidean space manifold. 

### Work with planners
SDCL has been tested for use with PRM-like planners. SDCL depends on the planner to properly add its samples to the `ompl::base::PlannerData` structure with the state and a tag. SDCL uses the tags to classify samples. Samples within the same CFree region should have the same tag. 


### Papers
- S. Li and N. Dantam. _Sample-Driven Connectivity Learning for Motion Planning in Narrow Passages_, ICRA 2023. Also available at <http://www.neil.dantam.name/papers/li2023sdcl.pdf>