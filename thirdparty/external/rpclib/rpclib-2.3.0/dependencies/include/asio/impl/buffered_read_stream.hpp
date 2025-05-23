//
// impl/buffered_read_stream.hpp
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2015 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef ASIO_IMPL_BUFFERED_READ_STREAM_HPP
#define ASIO_IMPL_BUFFERED_READ_STREAM_HPP

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
# pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "asio/detail/handler_alloc_helpers.hpp"
#include "asio/detail/handler_cont_helpers.hpp"
#include "asio/detail/handler_invoke_helpers.hpp"
#include "asio/detail/handler_type_requirements.hpp"

#include "asio/detail/push_options.hpp"

namespace clmdep_asio {

template <typename Stream>
std::size_t buffered_read_stream<Stream>::fill()
{
  detail::buffer_resize_guard<detail::buffered_stream_storage>
    resize_guard(storage_);
  std::size_t previous_size = storage_.size();
  storage_.resize(storage_.capacity());
  storage_.resize(previous_size + next_layer_.read_some(buffer(
          storage_.data() + previous_size,
          storage_.size() - previous_size)));
  resize_guard.commit();
  return storage_.size() - previous_size;
}

template <typename Stream>
std::size_t buffered_read_stream<Stream>::fill(clmdep_asio::error_code& ec)
{
  detail::buffer_resize_guard<detail::buffered_stream_storage>
    resize_guard(storage_);
  std::size_t previous_size = storage_.size();
  storage_.resize(storage_.capacity());
  storage_.resize(previous_size + next_layer_.read_some(buffer(
          storage_.data() + previous_size,
          storage_.size() - previous_size),
        ec));
  resize_guard.commit();
  return storage_.size() - previous_size;
}

namespace detail
{
  template <typename ReadHandler>
  class buffered_fill_handler
  {
  public:
    buffered_fill_handler(detail::buffered_stream_storage& storage,
        std::size_t previous_size, ReadHandler& handler)
      : storage_(storage),
        previous_size_(previous_size),
        handler_(ASIO_MOVE_CAST(ReadHandler)(handler))
    {
    }

#if defined(ASIO_HAS_MOVE)
    buffered_fill_handler(const buffered_fill_handler& other)
      : storage_(other.storage_),
        previous_size_(other.previous_size_),
        handler_(other.handler_)
    {
    }

    buffered_fill_handler(buffered_fill_handler&& other)
      : storage_(other.storage_),
        previous_size_(other.previous_size_),
        handler_(ASIO_MOVE_CAST(ReadHandler)(other.handler_))
    {
    }
#endif // defined(ASIO_HAS_MOVE)

    void operator()(const clmdep_asio::error_code& ec,
        const std::size_t bytes_transferred)
    {
      storage_.resize(previous_size_ + bytes_transferred);
      handler_(ec, bytes_transferred);
    }

  //private:
    detail::buffered_stream_storage& storage_;
    std::size_t previous_size_;
    ReadHandler handler_;
  };

  template <typename ReadHandler>
  inline void* clmdep_asio_handler_allocate(std::size_t size,
      buffered_fill_handler<ReadHandler>* this_handler)
  {
    return clmdep_asio_handler_alloc_helpers::allocate(
        size, this_handler->handler_);
  }

  template <typename ReadHandler>
  inline void clmdep_asio_handler_deallocate(void* pointer, std::size_t size,
      buffered_fill_handler<ReadHandler>* this_handler)
  {
    clmdep_asio_handler_alloc_helpers::deallocate(
        pointer, size, this_handler->handler_);
  }

  template <typename ReadHandler>
  inline bool clmdep_asio_handler_is_continuation(
      buffered_fill_handler<ReadHandler>* this_handler)
  {
    return clmdep_asio_handler_cont_helpers::is_continuation(
          this_handler->handler_);
  }

  template <typename Function, typename ReadHandler>
  inline void clmdep_asio_handler_invoke(Function& function,
      buffered_fill_handler<ReadHandler>* this_handler)
  {
    clmdep_asio_handler_invoke_helpers::invoke(
        function, this_handler->handler_);
  }

  template <typename Function, typename ReadHandler>
  inline void clmdep_asio_handler_invoke(const Function& function,
      buffered_fill_handler<ReadHandler>* this_handler)
  {
    clmdep_asio_handler_invoke_helpers::invoke(
        function, this_handler->handler_);
  }
} // namespace detail

template <typename Stream>
template <typename ReadHandler>
ASIO_INITFN_RESULT_TYPE(ReadHandler,
    void (clmdep_asio::error_code, std::size_t))
buffered_read_stream<Stream>::async_fill(
    ASIO_MOVE_ARG(ReadHandler) handler)
{
  // If you get an error on the following line it means that your handler does
  // not meet the documented type requirements for a ReadHandler.
  ASIO_READ_HANDLER_CHECK(ReadHandler, handler) type_check;

  detail::async_result_init<
    ReadHandler, void (clmdep_asio::error_code, std::size_t)> init(
      ASIO_MOVE_CAST(ReadHandler)(handler));

  std::size_t previous_size = storage_.size();
  storage_.resize(storage_.capacity());
  next_layer_.async_read_some(
      buffer(
        storage_.data() + previous_size,
        storage_.size() - previous_size),
      detail::buffered_fill_handler<ASIO_HANDLER_TYPE(
        ReadHandler, void (clmdep_asio::error_code, std::size_t))>(
        storage_, previous_size, init.handler));

  return init.result.get();
}

template <typename Stream>
template <typename MutableBufferSequence>
std::size_t buffered_read_stream<Stream>::read_some(
    const MutableBufferSequence& buffers)
{
  if (clmdep_asio::buffer_size(buffers) == 0)
    return 0;

  if (storage_.empty())
    this->fill();

  return this->copy(buffers);
}

template <typename Stream>
template <typename MutableBufferSequence>
std::size_t buffered_read_stream<Stream>::read_some(
    const MutableBufferSequence& buffers, clmdep_asio::error_code& ec)
{
  ec = clmdep_asio::error_code();

  if (clmdep_asio::buffer_size(buffers) == 0)
    return 0;

  if (storage_.empty() && !this->fill(ec))
    return 0;

  return this->copy(buffers);
}

namespace detail
{
  template <typename MutableBufferSequence, typename ReadHandler>
  class buffered_read_some_handler
  {
  public:
    buffered_read_some_handler(detail::buffered_stream_storage& storage,
        const MutableBufferSequence& buffers, ReadHandler& handler)
      : storage_(storage),
        buffers_(buffers),
        handler_(handler)
    {
    }

#if defined(ASIO_HAS_MOVE)
      buffered_read_some_handler(const buffered_read_some_handler& other)
        : storage_(other.storage_),
          buffers_(other.buffers_),
          handler_(other.handler_)
      {
      }

      buffered_read_some_handler(buffered_read_some_handler&& other)
        : storage_(other.storage_),
          buffers_(other.buffers_),
          handler_(ASIO_MOVE_CAST(ReadHandler)(other.handler_))
      {
      }
#endif // defined(ASIO_HAS_MOVE)

    void operator()(const clmdep_asio::error_code& ec, std::size_t)
    {
      if (ec || storage_.empty())
      {
        const std::size_t length = 0;
        handler_(ec, length);
      }
      else
      {
        const std::size_t bytes_copied = clmdep_asio::buffer_copy(
            buffers_, storage_.data(), storage_.size());
        storage_.consume(bytes_copied);
        handler_(ec, bytes_copied);
      }
    }

  //private:
    detail::buffered_stream_storage& storage_;
    MutableBufferSequence buffers_;
    ReadHandler handler_;
  };

  template <typename MutableBufferSequence, typename ReadHandler>
  inline void* clmdep_asio_handler_allocate(std::size_t size,
      buffered_read_some_handler<
        MutableBufferSequence, ReadHandler>* this_handler)
  {
    return clmdep_asio_handler_alloc_helpers::allocate(
        size, this_handler->handler_);
  }

  template <typename MutableBufferSequence, typename ReadHandler>
  inline void clmdep_asio_handler_deallocate(void* pointer, std::size_t size,
      buffered_read_some_handler<
        MutableBufferSequence, ReadHandler>* this_handler)
  {
    clmdep_asio_handler_alloc_helpers::deallocate(
        pointer, size, this_handler->handler_);
  }

  template <typename MutableBufferSequence, typename ReadHandler>
  inline bool clmdep_asio_handler_is_continuation(
      buffered_read_some_handler<
        MutableBufferSequence, ReadHandler>* this_handler)
  {
    return clmdep_asio_handler_cont_helpers::is_continuation(
          this_handler->handler_);
  }

  template <typename Function, typename MutableBufferSequence,
      typename ReadHandler>
  inline void clmdep_asio_handler_invoke(Function& function,
      buffered_read_some_handler<
        MutableBufferSequence, ReadHandler>* this_handler)
  {
    clmdep_asio_handler_invoke_helpers::invoke(
        function, this_handler->handler_);
  }

  template <typename Function, typename MutableBufferSequence,
      typename ReadHandler>
  inline void clmdep_asio_handler_invoke(const Function& function,
      buffered_read_some_handler<
        MutableBufferSequence, ReadHandler>* this_handler)
  {
    clmdep_asio_handler_invoke_helpers::invoke(
        function, this_handler->handler_);
  }
} // namespace detail

template <typename Stream>
template <typename MutableBufferSequence, typename ReadHandler>
ASIO_INITFN_RESULT_TYPE(ReadHandler,
    void (clmdep_asio::error_code, std::size_t))
buffered_read_stream<Stream>::async_read_some(
    const MutableBufferSequence& buffers,
    ASIO_MOVE_ARG(ReadHandler) handler)
{
  // If you get an error on the following line it means that your handler does
  // not meet the documented type requirements for a ReadHandler.
  ASIO_READ_HANDLER_CHECK(ReadHandler, handler) type_check;

  detail::async_result_init<
    ReadHandler, void (clmdep_asio::error_code, std::size_t)> init(
      ASIO_MOVE_CAST(ReadHandler)(handler));

  if (clmdep_asio::buffer_size(buffers) == 0 || !storage_.empty())
  {
    next_layer_.async_read_some(clmdep_asio::mutable_buffers_1(0, 0),
        detail::buffered_read_some_handler<
          MutableBufferSequence, ASIO_HANDLER_TYPE(
            ReadHandler, void (clmdep_asio::error_code, std::size_t))>(
            storage_, buffers, init.handler));
  }
  else
  {
    this->async_fill(detail::buffered_read_some_handler<
          MutableBufferSequence, ASIO_HANDLER_TYPE(
            ReadHandler, void (clmdep_asio::error_code, std::size_t))>(
            storage_, buffers, init.handler));
  }

  return init.result.get();
}

template <typename Stream>
template <typename MutableBufferSequence>
std::size_t buffered_read_stream<Stream>::peek(
    const MutableBufferSequence& buffers)
{
  if (storage_.empty())
    this->fill();
  return this->peek_copy(buffers);
}

template <typename Stream>
template <typename MutableBufferSequence>
std::size_t buffered_read_stream<Stream>::peek(
    const MutableBufferSequence& buffers, clmdep_asio::error_code& ec)
{
  ec = clmdep_asio::error_code();
  if (storage_.empty() && !this->fill(ec))
    return 0;
  return this->peek_copy(buffers);
}

} // namespace clmdep_asio

#include "asio/detail/pop_options.hpp"

#endif // ASIO_IMPL_BUFFERED_READ_STREAM_HPP
