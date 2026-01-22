#ifndef _DYNAMICAL_STATE_DATA_H
#define _DYNAMICAL_STATE_DATA_H

#include <map>
#include <vector>
#include <string>

#include <Vector.h>
#include <memory>

class DynamicalStateData {
public:
    using v_iter = std::map<std::string, std::vector<pba::Vector>>::iterator; 


    // Q: Can we still perform move operations since we are defining
    // a non default constructor??
    DynamicalStateData();
    ~DynamicalStateData() = default;

    std::vector<pba::Vector> get_positions();
    v_iter get_iterator(std::string name);

private:


};

using DynamicalStateData_sp = std::shared_ptr<DynamicalStateData>;

DynamicalStateData_sp create_dynamical_state_data() { return std::make_shared(new DynamicalStateData()) ;}


#endif