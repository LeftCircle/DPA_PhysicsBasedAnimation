#ifndef _SOFT_EDGE_H
#define _SOFT_EDGE_H


namespace pba{

class SoftEdge{
public:
	SoftEdge(size_t i, size_t j, double rest_len) : _index_a(i), _index_b(j), _rest_length(rest_len) {}

private:
	SoftEdge() = delete;

	size_t _index_a, _index_b;
	double _rest_length;

};


}// end namepspace pba




#endif