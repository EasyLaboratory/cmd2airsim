//
// impl/buffered_write_stream.hpp
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2015 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef ASIO_IMPL_BUFFERED_WRITE_STREAM_HPP
#define ASIO_IMPL_BUFFERED_WRITE_STREAM_HPP

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
std::size_t buffered_write_stream<Stream>::flush()
{
  std::size_t bytes_written = write(next_layer_,
      buffer(storage_.data(), storage_.size()));
  storage_.consume(bytes_written);
  return bytes_written;
}

template <typename Stream>
std::size_t buffered_write_stream<Stream>::flush(clmdep_asio::error_code& ec)
{
  std::size_t bytes_written = write(next_layer_,
      buffer(storage_.data(), storage_.size()),
      transfer_all(), ec);
  storage_.consume(bytes_written);
  return bytes_written;
}

namespace detail
{
  template <typename WriteHandler>
  class buffered_flush_handler
  {
  public:
    buffered_flush_handler(detail::buffered_stream_storage& storage,
        WriteHandler& handler)
      : storage_(storage),
        handler_(handler)
    {
    }

#if defined(ASIO_HAS_MOVE)
    buffered_flush_handler(const buffered_flush_handler& other)
      : storage_(other.storage_),
        handler_(other.handler_)
    {
    }

    buffered_flush_handler(buffered_flush_handler&& other)
      : storage_(other.storage_),
        handler_(ASIO_MOVE_CAST(WriteHandler)(other.handler_))
    {
    }
#endif // defined(ASIO_HAS_MOVE)

    void operator()(const clmdep_asio::error_code& ec,
        const std::size_t bytes_written)
    {
      storage_.consume(bytes_written);
      handler_(ec, bytes_written);
    }

  //private:
    detail::buffered_stream_storage& storage_;
    WriteHandler handler_;
  };

  template <typename WriteHandler>
  inline void* clmdep_asio_handler_allocate(std::size_t size,
      buffered_flush_handler<WriteHandler>* this_handler)
  {
    return clmdep_asio_handler_alloc_helpers::allocate(
        size, this_handler->handler_);
  }

  template <typename WriteHandler>
  inline void clmdep_asio_handler_deallocate(void* pointer, std::size_t size,
      buffered_flush_handler<WriteHandler>* this_handler)
  {
    clmdep_asio_handler_alloc_helpers::deallocate(
        pointer, size, this_handler->handler_);
  }

  template <typename WriteHandler>
  inline bool clmdep_asio_handler_is_continuation(
      buffered_flush_handler<WriteHandler>* this_handler)
  {
    return clmdep_asio_handler_cont_helpers::is_continuation(
          this_handler->handler_);
  }

  template <typename Function, typename WriteHandler>
  inline void clmdep_asio_handler_invoke(Function& function,
      buffered_flush_handler<WriteHandler>* this_handler)
  {
    clmdep_asio_handler_invoke_helpers::invoke(
        function, this_handler->handler_);
  }

  template <typename Function, typename WriteHandler>
  inline void clmdep_asio_handler_invoke(const Function& function,
      buffered_flush_handler<WriteHandler>* this_handler)
  {
    clmdep_asio_handler_invoke_helpers::invoke(
        function, this_handler->handler_);
  }
}

template <typename Stream>
template <typename WriteHandler>
ASIO_INITFN_RESULT_TYPE(WriteHandler,
    void (clmdep_asio::error_code, std::size_t))
buffered_write_stream<Stream>::async_flush(
    ASIO_MOVE_ARG(WriteHandler) handler)
{
  // If you get an error on the following line it means that your handler does
  // not meet the documented type requirements for a WriteHandler.
  ASIO_WRITE_HANDLER_CHECK(WriteHandler, handler) type_check;

  detail::async_result_init<
    WriteHandler, void (clmdep_asio::error_code, std::size_t)> init(
      ASIO_MOVE_CAST(WriteHandler)(handler));

  async_write(next_layer_, buffer(storage_.data(), storage_.size()),
      detail::buffered_flush_handler<ASIO_HANDLER_TYPE(
        WriteHandler, void (clmdep_asio::error_code, std::size_t))>(
        storage_, init.handler));

  return init.result.get();
}

template <typename Stream>
template <typename ConstBufferSequence>
std::size_t buffered_write_stream<Stream>::write_some(
    const ConstBufferSequence& buffers)
{
  if (clmdep_asio::buffer_size(buffers) == 0)
    return 0;

  if (storage_.size() == storage_.capacity())
    this->flush();

  return this->copy(buffers);
}

template <typename Stream>
template <typename ConstBufferSequence>
std::size_t buffered_write_stream<Stream>::write_some(
    const ConstBufferSequence& buffers, clmdep_asio::error_code& ec)
{
  ec = clmdep_asio::error_code();

  if (clmdep_asio::buffer_size(buffers) == 0)
    return 0;

  if (storage_.size() == storage_.capacity() && !flush(ec))
    return 0;

  return this->copy(buffers);
}

namespace detail
{
  template <typename ConstBufferSequence, typename WriteHandler>
  class buffered_write_some_handler
  {
  public:
    buffered_write_some_handler(detail::buffered_stream_storage& storage,
        const ConstBufferSequence& buffers, WriteHandler& handler)
      : storage_(storage),
        buffers_(buffers),
        handler_(handler)
    {
    }

#if defined(ASIO_HAS_MOVE)
      buffered_write_some_handler(const buffered_write_some_handler& other)
        : storage_(other.storage_),
          buffers_(other.buffers_),
          handler_(other.handler_)
      {
      }

      buffered_write_some_handler(buffered_write_some_handler&& other)
        : storage_(other.storage_),
          buffers_(other.buffers_),
          handler_(ASIO_MOVE_CAST(WriteHandler)(other.handler_))
      {
      }
#endif // defined(ASIO_HAS_MOVE)

    void operator()(const clmdep_asio::error_code& ec, std::size_t)
    {
      if (ec)
      {
        const std::size_t length = 0;
        handler_(ec, length);
      }
      else
      {
        std::size_t orig_size = storage_.size();
        std::size_t space_avail = storage_.capacity() - orig_size;
        std::size_t bytes_avail = clmdep_asio::buffer_size(buffers_);
        std::size_t length = bytes_avail < space_avail
          ? bytes_avail : space_avail;
        storage_.resize(orig_size + length);
        const std::size_t bytes_copied = clmdep_asio::buffer_copy(
            storage_.data() + orig_size, buffers_, length);
        handler_(ec, bytes_copied);
      }
    }

  //private:
    detail::buffered_stream_storage& storage_;
    ConstBufferSequence buffers_;
    WriteHandler handler_;
  };

  template <typename ConstBufferSequence, typename WriteHandler>
  inline void* clmdep_asio_handler_allocate(std::size_t size,
      buffered_write_some_handler<
        ConstBufferSequence, WriteHandler>* this_handler)
  {
    return clmdep_asio_handler_alloc_helpers::allocate(
        size, this_handler->handler_);
  }

  template <typename ConstBufferSequence, typename WriteHandler>
  inline void clmdep_asio_handler_deallocate(void* pointer, std::size_t size,
      buffered_write_some_handler<
        ConstBufferSequence, WriteHandler>* this_handler)
  {
    clmdep_asio_handler_alloc_helpers::deallocate(
        pointer, size, this_handler->handler_);
  }

  template <typename ConstBufferSequence, typename WriteHandler>
  inline bool clmdep_asio_handler_is_continuation(
      buffered_write_some_handler<
        ConstBufferSequence, WriteHandler>* this_handler)
  {
    return clmdep_asio_handler_cont_helpers::is_continuation(
          this_handler->handler_);
  }

  template <typename Function, typename ConstBufferSequence,
      typename WriteHandler>
  inline void clmdep_asio_handler_invoke(Function& function,
      buffered_write_some_handler<
        ConstBufferSequence, WriteHandler>* this_handler)
  {
    clmdep_asio_handler_invoke_helpers::invoke(
        function, this_handler->handler_);
  }

  template <typename Function, typename ConstBufferSequence,
      typename WriteHandler>
  inline void clmdep_asio_handler_invoke(const Function& function,
      buffered_write_some_handler<
        ConstBufferSequence, WriteHandler>* this_handler)
  {
    clmdep_asio_handler_invoke_helpers::invoke(
        function, this_handler->handler_);
  }
} // namespace detail

template <typename Stream>
template <typename ConstBufferSequence, typename WriteHandler>
ASIO_INITFN_RESULT_TYPE(WriteHandler,
    void (clmdep_asio::error_code, std::size_t))
buffered_write_stream<Stream>::async_write_some(
    const ConstBufferSequence& buffers,
    ASIO_MOVE_ARG(WriteHandler) handler)
{
  // If you get an error on the following line it means that your handler does
  // not meet the documented type requirements for a WriteHandler.
  ASIO_WRITE_HANDLER_CHECK(WriteHandler, handler) type_check;

  detail::async_result_init<
    WriteHandler, void (clmdep_asio::error_code, std::size_t)> init(
      ASIO_MOVE_CAST(WriteHandler)(handler));

  if (clmdep_asio::buffer_size(buffers) == 0
      || storage_.size() < storage_.capacity())
  {
    next_layer_.async_write_some(clmdep_asio::const_buffers_1(0, 0),
        detail::buffered_write_some_handler<
          ConstBufferSequence, ASIO_HANDLER_TYPE(
            WriteHandler, void (clmdep_asio::error_code, std::size_t))>(
            storage_, buffers, init.handler));
  }
  else
  {
    this->async_flush(detail::buffered_write_some_handler<
          ConstBufferSequence, ASIO_HANDLER_TYPE(
            WriteHandler, void (clmdep_asio::error_code, std::size_t))>(
            storage_, buffers, init.handler));
  }

  return init.result.get();
}

template <typename Stream>
template <typename ConstBufferSequence>
std::size_t buffered_write_stream<Stream>::copy(
    const ConstBufferSequence& buffers)
{
  std::size_t orig_size = storage_.size();
  std::size_t space_avail = storage_.capacity() - orig_size;
  std::size_t bytes_avail = clmdep_asio::buffer_size(buffers);
  std::size_t length = bytes_avail < space_avail ? bytes_avail : space_avail;
  storage_.resize(orig_size + length);
  return clmdep_asio::buffer_copy(
      storage_.data() + orig_size, buffers, length);
}

} // namespace clmdep_asio

#include "asio/detail/pop_options.hpp"

#endif // ASIO_IMPL_BUFFERED_WRITE_STREAM_HPP
