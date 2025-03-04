//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2015 KONDO Takatoshi
//
//    Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//    http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef MSGPACK_V1_TYPE_CPP11_REFERENCE_WRAPPER_HPP
#define MSGPACK_V1_TYPE_CPP11_REFERENCE_WRAPPER_HPP

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"
#include "rpc/msgpack/adaptor/check_container_size.hpp"

#include <memory>
#include <type_traits>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace adaptor {

template <typename T>
struct convert<std::reference_wrapper<T>> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, std::reference_wrapper<T>& v) const {
        clmdep_msgpack::adaptor::convert<T>()(o, v.get());
        return o;
    }
};

template <typename T>
struct pack<std::reference_wrapper<T>> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const std::reference_wrapper<T>& v) const {
        o.pack(v.get());
        return o;
    }
};

template <typename T>
struct object<std::reference_wrapper<T> > {
    void operator()(clmdep_msgpack::object& o, const std::reference_wrapper<T>& v) const {
        clmdep_msgpack::adaptor::object<typename std::remove_const<T>::type>()(o, v.get());
    }
};

template <typename T>
struct object_with_zone<std::reference_wrapper<T>> {
    void operator()(clmdep_msgpack::object::with_zone& o, const std::reference_wrapper<T>& v) const {
        clmdep_msgpack::adaptor::object_with_zone<typename std::remove_const<T>::type>()(o, v.get());
    }
};

} // namespace adaptor

/// @cond
} // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

} // namespace clmdep_msgpack

#endif // MSGPACK_V1_TYPE_CPP11_REFERENCE_WRAPPER_HPP
