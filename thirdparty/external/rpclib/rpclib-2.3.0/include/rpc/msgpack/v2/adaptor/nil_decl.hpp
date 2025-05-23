//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2008-2016 FURUHASHI Sadayuki and KONDO Takatoshi
//
//    Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//    http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef MSGPACK_V2_TYPE_NIL_DECL_HPP
#define MSGPACK_V2_TYPE_NIL_DECL_HPP

#include "rpc/msgpack/v1/adaptor/nil_decl.hpp"

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v2) {
/// @endcond

namespace type {

using v1::type::nil_t;

#if defined(MSGPACK_USE_LEGACY_NIL)

typedef nil_t nil;

#endif // defined(MSGPACK_USE_LEGACY_NIL)

using v1::type::operator<;
using v1::type::operator==;

}  // namespace type

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v2)
/// @endcond

}  // namespace clmdep_msgpack

#endif // MSGPACK_V2_TYPE_NIL_DECL_HPP
