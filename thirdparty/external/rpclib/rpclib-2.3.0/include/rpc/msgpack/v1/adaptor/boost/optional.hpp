//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2015 KONDO Takatoshi
//
//    Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//    http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef MSGPACK_V1_TYPE_BOOST_OPTIONAL_HPP
#define MSGPACK_V1_TYPE_BOOST_OPTIONAL_HPP

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"
#include "rpc/msgpack/adaptor/check_container_size.hpp"

// To suppress warning on Boost.1.58.0
#if (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif // (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)) || defined(__clang__)

#include <boost/optional.hpp>

#if (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)) || defined(__clang__)
#pragma GCC diagnostic pop
#endif // (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)) || defined(__clang__)

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace adaptor {

#if !defined (MSGPACK_USE_CPP03)

template <typename T>
struct as<boost::optional<T>, typename std::enable_if<clmdep_msgpack::has_as<T>::value>::type> {
    boost::optional<T> operator()(clmdep_msgpack::object const& o) const {
        if(o.is_nil()) return boost::none;
        return o.as<T>();
    }
};

#endif // !defined (MSGPACK_USE_CPP03)

template <typename T>
struct convert<boost::optional<T> > {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, boost::optional<T>& v) const {
        if(o.is_nil()) v = boost::none;
        else {
            T t;
            clmdep_msgpack::adaptor::convert<T>()(o, t);
            v = t;
        }
        return o;
    }
};

template <typename T>
struct pack<boost::optional<T> > {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const boost::optional<T>& v) const {
        if (v) o.pack(*v);
        else o.pack_nil();
        return o;
    }
};

template <typename T>
struct object<boost::optional<T> > {
    void operator()(clmdep_msgpack::object& o, const boost::optional<T>& v) const {
        if (v) clmdep_msgpack::adaptor::object<T>()(o, *v);
        else o.type = clmdep_msgpack::type::NIL;
    }
};

template <typename T>
struct object_with_zone<boost::optional<T> > {
    void operator()(clmdep_msgpack::object::with_zone& o, const boost::optional<T>& v) const {
        if (v) clmdep_msgpack::adaptor::object_with_zone<T>()(o, *v);
        else o.type = clmdep_msgpack::type::NIL;
    }
};

} // namespace adaptor

/// @cond
} // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

} // namespace clmdep_msgpack

#endif // MSGPACK_V1_TYPE_BOOST_OPTIONAL_HPP
