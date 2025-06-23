#pragma once

template <class T> 
constexpr T lerp(T x, T a, T b)
{
    return x*(b-a)+a;
}

template <class T> 
constexpr T inverse_lerp(T y, T a, T b)
{
    return (y-a)/(b-a);
}

template <class T> 
constexpr T map(T x, T ax, T bx, T ay, T by)
{
    return lerp<T>(inverse_lerp<T>(x,ax,bx),ay,by);
}