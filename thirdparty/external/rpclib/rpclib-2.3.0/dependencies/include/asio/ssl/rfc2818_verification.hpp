//
// ssl/rfc2818_verification.hpp
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2015 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef ASIO_SSL_RFC2818_VERIFICATION_HPP
#define ASIO_SSL_RFC2818_VERIFICATION_HPP

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
# pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "asio/detail/config.hpp"

#if !defined(ASIO_ENABLE_OLD_SSL)
# include <string>
# include "asio/ssl/detail/openssl_types.hpp"
# include "asio/ssl/verify_context.hpp"
#endif // !defined(ASIO_ENABLE_OLD_SSL)

#include "asio/detail/push_options.hpp"

namespace clmdep_asio {
namespace ssl {

#if !defined(ASIO_ENABLE_OLD_SSL)

/// Verifies a certificate against a hostname according to the rules described
/// in RFC 2818.
/**
 * @par Example
 * The following example shows how to synchronously open a secure connection to
 * a given host name:
 * @code
 * using clmdep_asio::ip::tcp;
 * namespace ssl = clmdep_asio::ssl;
 * typedef ssl::stream<tcp::socket> ssl_socket;
 *
 * // Create a context that uses the default paths for finding CA certificates.
 * ssl::context ctx(ssl::context::sslv23);
 * ctx.set_default_verify_paths();
 *
 * // Open a socket and connect it to the remote host.
 * clmdep_asio::io_service io_service;
 * ssl_socket sock(io_service, ctx);
 * tcp::resolver resolver(io_service);
 * tcp::resolver::query query("host.name", "https");
 * clmdep_asio::connect(sock.lowest_layer(), resolver.resolve(query));
 * sock.lowest_layer().set_option(tcp::no_delay(true));
 *
 * // Perform SSL handshake and verify the remote host's certificate.
 * sock.set_verify_mode(ssl::verify_peer);
 * sock.set_verify_callback(ssl::rfc2818_verification("host.name"));
 * sock.handshake(ssl_socket::client);
 *
 * // ... read and write as normal ...
 * @endcode
 */
class rfc2818_verification
{
public:
  /// The type of the function object's result.
  typedef bool result_type;

  /// Constructor.
  explicit rfc2818_verification(const std::string& host)
    : host_(host)
  {
  }

  /// Perform certificate verification.
  ASIO_DECL bool operator()(bool preverified, verify_context& ctx) const;

private:
  // Helper function to check a host name against a pattern.
  ASIO_DECL static bool match_pattern(const char* pattern,
      std::size_t pattern_length, const char* host);

  // Helper function to check a host name against an IPv4 address
  // The host name to be checked.
  std::string host_;
};

#endif // defined(ASIO_ENABLE_OLD_SSL)

} // namespace ssl
} // namespace clmdep_asio

#include "asio/detail/pop_options.hpp"

#if defined(ASIO_HEADER_ONLY)
# include "asio/ssl/impl/rfc2818_verification.ipp"
#endif // defined(ASIO_HEADER_ONLY)

#endif // ASIO_SSL_RFC2818_VERIFICATION_HPP
