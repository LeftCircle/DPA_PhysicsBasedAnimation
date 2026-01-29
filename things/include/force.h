#ifndef _FORCE_H
#define _FORCE_H

#include <memory>

#include "dynamical_state_data.h"

namespace pba{

class ForceBase{

public:
	ForceBase() = default;
	virtual ~ForceBase() = default;
	virtual void compute(DynamicalStateData_sp dsd, const double dt) const = 0;
};
using Force_sp = std::shared_ptr<ForceBase>;

class ForceSystem : public ForceBase{
public:

	ForceSystem() = default;
	~ForceSystem() = default;
	void add_force(Force_sp force) { _forces.push_back(force); }

	void compute(DynamicalStateData_sp dsd, const double dt) const override;

private:
	void _reset_accelerations(DynamicalStateData_sp dsd) const noexcept;
	std::vector<Force_sp> _forces;
};

using ForceSystem_sp = std::shared_ptr<ForceSystem>;

} // end namespace pba


#endif