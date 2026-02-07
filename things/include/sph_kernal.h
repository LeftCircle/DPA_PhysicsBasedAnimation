#ifndef SPH_KERNAL_H
#define SPH_KERNAL_H

#include "Vector.h"

const double PId = 3.14159265358979;
// An sph kernal defines a curve with an area of one which represents the influence of 
// a particle on the area around it. 

// This is the basic kernal implementation from Doyub Kim's fluid engine development book
template<typename Vec>
struct SphStdKernal3 {
    // 315 / (64 * pi * h^3) (1 - (r^2 / h^2)^3) when 0 <= r < = h
    double h1, h2, h3, h5;

    SphStdKernal3() : h1(0), h2(0), h3(0), h5(0) {};
    explicit SphStdKernal3(double radius) : h1(radius), h2(radius * radius), h3(h2 * radius), h5(h2 * h3) {};

    double operator()(double distance) const noexcept;

    double first_derivative(double distance) const noexcept;
    double second_derivative(double distance) const noexcept;
    Vec gradient(double distance, const Vec& direction) const noexcept;
};

template<typename T>
struct SphStdKernal2 {
    T h, h2, h3, h4;

    SphStdKernal2() : h(0), h2(0), h3(0), h4(0) {};
    explicit SphStdKernal2(T radius) : h(radius), h2(radius * radius), h3(h2 * radius), h4(h2 * h2) {};

    T operator()(T distance) const;

    T first_derivative(T distance) const;
    T second_derivative(T distance) const;
    cato::Vec2T<T> gradient(T distance, const cato::Vec2T<T>& direction) const;
};

template<typename T>
struct SphSpikyKernal3 {
    T h1, h2, h3, h4, h5;

    SphSpikyKernal3() : h1(0), h2(0), h3(0), h4(0), h5(0) {};
    explicit SphSpikyKernal3(T radius) : h1(radius), h2 (h1 * h1), h3(h1 * h2), h4(h2 * h2), h5(h3 * h2) {};

    T operator()(T distance) const;
    T first_derivative(T distance) const;
    T second_derivative(T distance) const;
    cato::Vec3T<T> gradient(T distance, const cato::Vec3T<T>& direction) const;
};

template<typename T>
struct SphSpikyKernal2 {
    T h1, h2, h3, h4;

    SphSpikyKernal2() : h1(0), h2(0), h3(0), h4(0) {};
    explicit SphSpikyKernal2(T radius) : h1(radius), h2(radius * radius), h3(radius * h2), h4(h2 * h2) {};

    T operator()(T distance) const;
    T first_derivative(T distance) const;
    T second_derivative(T distance) const;
    cato::Vec2T<T> gradient(T distance, const cato::Vec2T<T>& direction) const;
};


template<typename T>
inline T SphStdKernal3<T>::operator()(T distance) const{
    if (distance * distance >= h2){
        return 0.0;
    } else {
        T x = (1.0 - (distance * distance) / h2);
        return 315.0 / (64.0 * PId * h3) * x * x * x;
    }
}

template<typename T>
inline T SphStdKernal3<T>::first_derivative(T distance) const {
    if (distance * distance >= h2){
        return 0.0;
    } else {
        T x = (1 - distance * distance / h2);
        return -945.0 / (32.0 * PId * h5) * distance * x * x;
    }
}

template<typename T>
inline T SphStdKernal3<T>::second_derivative(T distance) const {
    if (distance * distance >= h2){
        return 0.0;
    } else {
        T x = distance * distance / h2;
        return 945.0 / (32.0 * PId * h5) * (3 * x - 1) * (1 - x);
    }
}

template <typename T>
inline cato::Vec3T<T> SphStdKernal3<T>::gradient(T distance, const cato::Vec3T<T>& direction) const {
    // Direction is assumed to be normalized
    return -first_derivative(distance) * direction;
}

template<typename T>
inline T SphStdKernal2<T>::operator()(T distance) const {
    if (distance >= h){
        return 0.0;
    } else {
        T x = (1 - distance * distance / h2);
        return (4.0 / (PId * h2)) * x * x * x;

    }
}

template<typename T>
inline T SphStdKernal2<T>::first_derivative(T distance) const {
    if (distance >= h){
        return 0.0;
    } else {
        T x = (1 - distance * distance / h2);
        return -24.0 / (PId * h2 * h2) * distance * x * x;
    }
}

template<typename T>
inline T SphStdKernal2<T>::second_derivative(T distance) const {
    if (distance >= h){
        return 0.0;
    } else {
        T x = distance * distance / h2;
        return 24.0 / (PId * h4) * (1 - x) * (5 * x - 1);
    }
}

template<typename T>
inline cato::Vec2T<T> SphStdKernal2<T>::gradient(T distance, const cato::Vec2T<T>& direction) const {
    // Direction is assumed to be normalized
    return -first_derivative(distance) * direction;
}

template<typename T>
inline T SphSpikyKernal3<T>::operator()(T distance) const{
    if (distance >= h1){
        return 0.0;
    } else {
        T x = (1.0 - (distance) / h1);
        return 15.0 / (PId * h3) * x * x * x;
    }
}

template<typename T>
inline T SphSpikyKernal3<T>::first_derivative(T distance) const {
    if (distance >= h1){
        return 0.0;
    } else {
        T x = (1.0 - (distance) / h1);
        return -45.0 / (PId * h4) * x * x;
    }
}

template<typename T>
inline T SphSpikyKernal3<T>::second_derivative(T distance) const {
    if (distance >= h1){
        return 0.0;
    } else {
        T x = (1.0 - (distance) / h1);
        return 90.0 / (PId * h5) * x;
    }
}

template<typename T>
inline cato::Vec3T<T> SphSpikyKernal3<T>::gradient(T distance, const cato::Vec3T<T>& direction) const {
    // Direction is assumed to be normalized
    return -first_derivative(distance) * direction;
}

template<typename T>
inline T SphSpikyKernal2<T>::operator()(T distance) const{
    if (distance * distance >= h2){
        return 0.0;
    } else {
        T x = 1.0 - distance / h1;
        return 10.0 / (PId * h2) * x * x * x;
    }
}

template<typename T>
inline T SphSpikyKernal2<T>::first_derivative(T distance) const {
    if (distance >= h1){
        return 0.0;
    } else {
        T x = (1.0 - (distance) / h1);
        return -30.0 / (PId * h3) * x * x;
    }
}

template<typename T>
inline T SphSpikyKernal2<T>::second_derivative(T distance) const {
    if (distance >= h1){
        return 0.0;
    } else {
        T x = (1.0 - (distance) / h1);
        return 60.0 / (PId * h4) * x;
    }
}

template<typename T>
inline cato::Vec2T<T> SphSpikyKernal2<T>::gradient(T distance, const cato::Vec2T<T>& direction) const {
    // Direction is assumed to be normalized
    return -first_derivative(distance) * direction;
}

using SphStdKernal3d = SphStdKernal3<double>;
using SphStdKernal3f = SphStdKernal3<float>;
using SphStdKernal2d = SphStdKernal2<double>;
using SphStdKernal2f = SphStdKernal2<float>;


#endif