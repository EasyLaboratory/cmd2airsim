//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2016 KONDO Takatoshi
//
//    Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//    http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef MSGPACK_V1_DEFINE_DECL_HPP
#define MSGPACK_V1_DEFINE_DECL_HPP

#include "rpc/msgpack/cpp_config.hpp"

#if defined(MSGPACK_USE_CPP03)
#include "rpc/msgpack/v1/adaptor/detail/cpp03_define_array_decl.hpp"
#include "rpc/msgpack/v1/adaptor/detail/cpp03_define_map_decl.hpp"
#else  // MSGPACK_USE_CPP03
#include "rpc/msgpack/v1/adaptor/detail/cpp11_define_array_decl.hpp"
#include "rpc/msgpack/v1/adaptor/detail/cpp11_define_map_decl.hpp"
#endif // MSGPACK_USE_CPP03

#endif // MSGPACK_V1_DEFINE_DECL_HPP
