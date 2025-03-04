# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Edward Diener 2014.
#  *     Distributed under the Boost Software License, Version 1.0. (See
#  *     accompanying file LICENSE_1_0.txt or copy at
#  *     http://www.boost.org/LICENSE_1_0.txt)
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef MSGPACK_PREPROCESSOR_FACILITIES_IS_EMPTY_VARIADIC_HPP
# define MSGPACK_PREPROCESSOR_FACILITIES_IS_EMPTY_VARIADIC_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# if MSGPACK_PP_VARIADICS
#
# include <rpc/msgpack/preprocessor/punctuation/is_begin_parens.hpp>
# include <rpc/msgpack/preprocessor/facilities/detail/is_empty.hpp>
#
#if MSGPACK_PP_VARIADICS_MSVC && _MSC_VER <= 1400
#
#define MSGPACK_PP_IS_EMPTY(param) \
    MSGPACK_PP_DETAIL_IS_EMPTY_IIF \
      ( \
      MSGPACK_PP_IS_BEGIN_PARENS \
        ( \
        param \
        ) \
      ) \
      ( \
      MSGPACK_PP_IS_EMPTY_ZERO, \
      MSGPACK_PP_DETAIL_IS_EMPTY_PROCESS \
      ) \
    (param) \
/**/
#define MSGPACK_PP_IS_EMPTY_ZERO(param) 0
# else
#define MSGPACK_PP_IS_EMPTY(...) \
    MSGPACK_PP_DETAIL_IS_EMPTY_IIF \
      ( \
      MSGPACK_PP_IS_BEGIN_PARENS \
        ( \
        __VA_ARGS__ \
        ) \
      ) \
      ( \
      MSGPACK_PP_IS_EMPTY_ZERO, \
      MSGPACK_PP_DETAIL_IS_EMPTY_PROCESS \
      ) \
    (__VA_ARGS__) \
/**/
#define MSGPACK_PP_IS_EMPTY_ZERO(...) 0
# endif /* MSGPACK_PP_VARIADICS_MSVC && _MSC_VER <= 1400 */
# endif /* MSGPACK_PP_VARIADICS */
# endif /* MSGPACK_PREPROCESSOR_FACILITIES_IS_EMPTY_VARIADIC_HPP */
