#ifndef _CATO_SPAN__
#define _CATO_SPAN__

#include <vector>
#include <memory>

#if __cplusplus < 202002L
namespace cato{

template <typename T>
class span {
public:
	span(T* data, size_t size) : _data(data), _size(size) {};
	
	// the constructors for std::vectors
	// for span<T>
	template <typename U = T,
		std::enable_if_t<!std::is_const<U>::value, bool> = true>
	span(std::vector<U>& vec) : _data(vec.data()), _size(vec.size()) {};

	// for span<const T>
	template <typename U,
		std::enable_if_t<std::is_same<T, const U>, bool> = true>
	span(const std::vector<U>& vec) : _data(vec.data()), _size(vec.size()) {}; 

	// Conversion from span<t> to span<const T>
	template <typename U,
		std::enable_if_t<std::is_same<T, const U>, bool> = true>
	span(const span<U>& other) : _data(other.data()), _size(other.size()){};


	T& operator[](size_t i) const { return _data[i]; }
	size_t size() const { return _size; }


private:
	// We want a raw pointer here because a span is a lightweight
	// non owning view into the vector
	T* _data;
	size_t _size;
};


}// end namespace cato


template <typename T>
using span = cato::span<T>;

#else
#include <span>
template <typename T>
using span = std::span<T>;
#endif


#endif