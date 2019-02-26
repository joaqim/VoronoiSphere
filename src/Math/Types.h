#pragma once

#include "xs_Float.h"

#if 0
#ifndef _xs_Types_
#define _xs_Types_
#include <stdint.h>
typedef char int8;
typedef unsigned char uint8;
typedef short int16;
typedef unsigned short uint16;
typedef long int32;
typedef unsigned long uint32;
typedef float real32;
typedef double real64;
#endif
#endif

#include <Magnum/Types.h>
typedef Magnum::UnsignedByte UnsignedByte; 
typedef Magnum::UnsignedShort UnsignedShort; 
typedef Magnum::UnsignedInt UnsignedInt; 
#if !defined(CORRADE_TARGET_EMSCRIPTEN) //TODO: Long and UnsignedLong (64-bit integers) are not supported on Emscripten
typedef Magnum::UnsignedLong UnsignedLong; 
#else
typedef Magnum::UnsignedInt UnsignedLong; 
#endif

#include <Magnum/Math/Vector.h>

typedef Magnum::Math::Rad<double> Rad;
typedef Magnum::Math::Rad<float> Radf;

typedef Magnum::Math::Vector2<float> Vector2f;
typedef Magnum::Math::Vector3<float> Vector3f;
typedef Magnum::Math::Vector4<float> Vector4f;
typedef Magnum::Math::Matrix4<float> Matrix4f;

typedef Magnum::Math::Vector2<double> Vector2d;
typedef Magnum::Math::Vector3<double> Vector3d;
typedef Magnum::Math::Vector4<double> Vector4d;
typedef Magnum::Math::Matrix4<double> Matrix4d;

#include <Magnum/Math/Color.h>

typedef Magnum::Math::Color3<float> Color3;
typedef Magnum::Math::Color4<float> Color4;

