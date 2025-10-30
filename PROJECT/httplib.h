//  httplib.h
//
//  Copyright (c) 2023 Yuji Hirose. All rights reserved.
//  MIT License
//

#ifndef CPPHTTPLIB_HTTPLIB_H
#define CPPHTTPLIB_HTTPLIB_H

#define CPPHTTPLIB_VERSION "0.14.1"

#ifdef _WIN32
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif //_CRT_SECURE_NO_WARNINGS

#ifndef _CRT_NONSTDC_NO_DEPRECATE
#define _CRT_NONSTDC_NO_DEPRECATE
#endif //_CRT_NONSTDC_NO_DEPRECATE

#if defined(_MSC_VER)
#if _MSC_VER < 1900
#error Sorry, Visual Studio versions prior to 2015 are not supported
#endif

#pragma comment(lib, "ws2_32.lib")

#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0601
#endif //_WIN32_WINNT

#include <fcntl.h>
#include <io.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#ifndef WSA_FLAG_NO_HANDLE_INHERIT
#define WSA_FLAG_NO_HANDLE_INHERIT 0x80
#endif

#ifdef _MSC_VER
#pragma comment(lib, "crypt32.lib")
#pragma comment(lib, "cryptui.lib")
#endif
#elif defined(__CYGWIN__)
#define _POSIX_C_SOURCE 200809L
#endif // _WIN32

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cctype>
#include <climits>
#include <condition_variable>
#include <cstring>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
#ifdef _WIN32
#include <wincrypt.h>

#ifdef _MSC_VER
#pragma comment(lib, "crypt32.lib")
#pragma comment(lib, "cryptui.lib")
#endif
#elif defined(CPPHTTPLIB_USE_CERTS_FROM_MACOSX_KEYCHAIN) && defined(__APPLE__)
#include <TargetConditionals.h>
#if TARGET_OS_OSX
#include <CoreFoundation/CoreFoundation.h>
#include <Security/Security.h>
#endif // TARGET_OS_OSX
#endif // _WIN32

#include <openssl/err.h>
#include <openssl/evp.h>
#include <openssl/ssl.h>
#include <openssl/x509v3.h>

#if defined(_WIN32) && defined(OPENSSL_USE_APPLINK)
#include <openssl/applink.c>
#endif

#include <openssl/crypto.h>

#if OPENSSL_VERSION_NUMBER < 0x1010100fL
#error Sorry, OpenSSL versions prior to 1.1.1 are not supported
#elif OPENSSL_VERSION_NUMBER < 0x10100000L
#define SSL_get1_peer_certificate SSL_get_peer_certificate
#endif

#endif

#ifdef CPPHTTPLIB_ZLIB_SUPPORT
#include <zlib.h>
#endif

#ifdef CPPHTTPLIB_BROTLI_SUPPORT
#include <brotli/decode.h>
#include <brotli/encode.h>
#endif

/*
 * Configuration
 */

#ifndef CPPHTTPLIB_KEEPALIVE_TIMEOUT_SECOND
#define CPPHTTPLIB_KEEPALIVE_TIMEOUT_SECOND 5
#endif

#ifndef CPPHTTPLIB_KEEPALIVE_MAX_COUNT
#define CPPHTTPLIB_KEEPALIVE_MAX_COUNT 5
#endif

#ifndef CPPHTTPLIB_CONNECTION_TIMEOUT_SECOND
#define CPPHTTPLIB_CONNECTION_TIMEOUT_SECOND 300
#endif

#ifndef CPPHTTPLIB_CONNECTION_TIMEOUT_USECOND
#define CPPHTTPLIB_CONNECTION_TIMEOUT_USECOND 0
#endif

#ifndef CPPHTTPLIB_READ_TIMEOUT_SECOND
#define CPPHTTPLIB_READ_TIMEOUT_SECOND 5
#endif

#ifndef CPPHTTPLIB_READ_TIMEOUT_USECOND
#define CPPHTTPLIB_READ_TIMEOUT_USECOND 0
#endif

#ifndef CPPHTTPLIB_WRITE_TIMEOUT_SECOND
#define CPPHTTPLIB_WRITE_TIMEOUT_SECOND 5
#endif

#ifndef CPPHTTPLIB_WRITE_TIMEOUT_USECOND
#define CPPHTTPLIB_WRITE_TIMEOUT_USECOND 0
#endif

#ifndef CPPHTTPLIB_IDLE_INTERVAL_SECOND
#define CPPHTTPLIB_IDLE_INTERVAL_SECOND 0
#endif

#ifndef CPPHTTPLIB_IDLE_INTERVAL_USECOND
#define CPPHTTPLIB_IDLE_INTERVAL_USECOND 0
#endif

#ifndef CPPHTTPLIB_REQUEST_URI_MAX_LENGTH
#define CPPHTTPLIB_REQUEST_URI_MAX_LENGTH 8192
#endif

#ifndef CPPHTTPLIB_HEADER_MAX_LENGTH
#define CPPHTTPLIB_HEADER_MAX_LENGTH 8192
#endif

#ifndef CPPHTTPLIB_REDIRECT_MAX_COUNT
#define CPPHTTPLIB_REDIRECT_MAX_COUNT 20
#endif

#ifndef CPPHTTPLIB_MULTIPART_FORM_DATA_FILE_MAX_COUNT
#define CPPHTTPLIB_MULTIPART_FORM_DATA_FILE_MAX_COUNT 1024
#endif

#ifndef CPPHTTPLIB_PAYLOAD_MAX_LENGTH
#define CPPHTTPLIB_PAYLOAD_MAX_LENGTH ((std::numeric_limits<size_t>::max)())
#endif

#ifndef CPPHTTPLIB_FORM_URL_ENCODED_PAYLOAD_MAX_LENGTH
#define CPPHTTPLIB_FORM_URL_ENCODED_PAYLOAD_MAX_LENGTH 8192
#endif

#ifndef CPPHTTPLIB_TCP_NODELAY
#define CPPHTTPLIB_TCP_NODELAY false
#endif

#ifndef CPPHTTPLIB_RECV_BUFSIZ
#define CPPHTTPLIB_RECV_BUFSIZ size_t(4096u)
#endif

#ifndef CPPHTTPLIB_COMPRESSION_BUFSIZ
#define CPPHTTPLIB_COMPRESSION_BUFSIZ size_t(16384u)
#endif

#ifndef CPPHTTPLIB_THREAD_POOL_COUNT
// if hardware_concurrency() is not well supported, make thread pool size 8
#define CPPHTTPLIB_THREAD_POOL_COUNT                                           \
  ((std::thread::hardware_concurrency() > 0)                                  \
       ? std::thread::hardware_concurrency()                                  \
       : 8)
#endif

#ifndef CPPHTTPLIB_RECV_FLAGS
#define CPPHTTPLIB_RECV_FLAGS 0
#endif

#ifndef CPPHTTPLIB_SEND_FLAGS
#define CPPHTTPLIB_SEND_FLAGS 0
#endif

#ifndef CPPHTTPLIB_LISTEN_BACKLOG
#define CPPHTTPLIB_LISTEN_BACKLOG 5
#endif

/*
 * Headers
 */

#ifdef _WIN32
#ifndef _WINSOCK_DEPRECATED_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#endif
#include <winsock2.h>
#include <ws2tcpip.h>

#ifndef strcasecmp
#define strcasecmp _stricmp
#endif // strcasecmp

using socket_t = SOCKET;
#ifdef CPPHTTPLIB_USE_POLL
#define poll(fds, nfds, timeout) WSAPoll(fds, nfds, timeout)
#endif

#else // not _WIN32

#include <arpa/inet.h>
#if !defined(_AIX) && !defined(__MVS__)
#include <ifaddrs.h>
#endif
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#ifdef __linux__
#include <resolv.h>
#endif
#include <netinet/tcp.h>
#ifdef CPPHTTPLIB_USE_POLL
#include <poll.h>
#endif
#include <csignal>
#include <pthread.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

using socket_t = int;
#ifndef INVALID_SOCKET
#define INVALID_SOCKET (-1)
#endif
#endif //_WIN32

#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <regex>
#include <string>
#include <thread>
#include <unordered_map>

#ifndef CPPHTTPLIB_HTTPLIB_H
#define CPPHTTPLIB_HTTPLIB_H

namespace httplib {

namespace detail {

/*
 * Backport std::make_unique from C++14.
 *
 * NOTE: This code came up with the following stackoverflow post:
 * https://stackoverflow.com/questions/10149840/c-arrays-and-make-unique
 *
 */

template <class T, class... Args>
typename std::enable_if<!std::is_array<T>::value, std::unique_ptr<T>>::type
make_unique(Args &&...args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

template <class T>
typename std::enable_if<std::is_array<T>::value, std::unique_ptr<T>>::type
make_unique(std::size_t n) {
  typedef typename std::remove_extent<T>::type RT;
  return std::unique_ptr<T>(new RT[n]);
}

struct ci {
  bool operator()(const std::string &s1, const std::string &s2) const {
    return std::lexicographical_compare(s1.begin(), s1.end(), s2.begin(),
                                        s2.end(),
                                        [](unsigned char c1, unsigned char c2) {
                                          return ::tolower(c1) < ::tolower(c2);
                                        });
  }
};

} // namespace detail

using Headers = std::multimap<std::string, std::string, detail::ci>;

using Params = std::multimap<std::string, std::string>;
using Match = std::smatch;

using Progress = std::function<bool(uint64_t current, uint64_t total)>;

struct Response;
using ResponseHandler = std::function<bool(const Response &response)>;

using ContentReader = std::function<bool(ContentReceiver &receiver)>;

using ContentWriter =
    std::function<bool(size_t offset, size_t length, DataSink &sink)>;

using ContentProviderWithoutLength =
    std::function<bool(size_t offset, DataSink &sink)>;

using ContentProviderResourceReleaser = std::function<void(bool success)>;

struct MultipartFormData {
  std::string name;
  std::string content;
  std::string filename;
  std::string content_type;
};
using MultipartFormDataItems = std::vector<MultipartFormData>;
using MultipartFormDataMap = std::multimap<std::string, MultipartFormData>;

class DataSink {
public:
  DataSink() : os(&sb_), sb_(*this) {}

  DataSink(const DataSink &) = delete;
  DataSink &operator=(const DataSink &) = delete;
  DataSink(DataSink &&) = delete;
  DataSink &operator=(DataSink &&) = delete;

  std::function<bool(const char *data, size_t data_len)> write;
  std::function<void()> done;
  std::function<void()> done_with_trailer;
  std::ostream os;

private:
  class data_sink_streambuf : public std::streambuf {
  public:
    explicit data_sink_streambuf(DataSink &sink) : sink_(sink) {}

  protected:
    std::streamsize xsputn(const char *s, std::streamsize n) override {
      sink_.write(s, static_cast<size_t>(n));
      return n;
    }

  private:
    DataSink &sink_;
  };

  data_sink_streambuf sb_;
};

using ContentReceiver =
    std::function<bool(const char *data, size_t data_length)>;

using MultipartContentHeader =
    std::function<bool(const MultipartFormData &file)>;

class ContentProvider {
public:
  using Reader = std::function<bool(size_t offset, size_t length, DataSink &sink)>;
  using Closer = std::function<void(bool success)>;

  ContentProvider(Reader reader, Closer closer)
      : reader_(std::move(reader)), closer_(std::move(closer)) {}

  ContentProvider(Reader reader)
      : reader_(std::move(reader)), closer_([](bool) {}) {}

  ContentProvider() = default;
  ContentProvider(const ContentProvider &) = delete;
  ContentProvider &operator=(const ContentProvider &) = delete;
  ContentProvider(ContentProvider &&) = default;
  ContentProvider &operator=(ContentProvider &&) = default;

  virtual ~ContentProvider() { closer_(false); }

  bool operator()(size_t offset, size_t length, DataSink &sink) const {
    return reader_(offset, length, sink);
  }

  void close() { closer_(true); }

private:
  Reader reader_;
  Closer closer_;
};

struct Request {
  std::string method;
  std::string path;
  Headers headers;
  std::string body;

  std::string remote_addr;
  int remote_port = -1;
  std::string local_addr;
  int local_port = -1;

  // for server
  std::string version;
  std::string target;
  Params params;
  MultipartFormDataMap files;
  Ranges ranges;
  Match matches;

  // for client
  ResponseHandler response_handler;
  ContentReceiver content_receiver;
  MultipartContentHeader multipart_content_header;
  Progress progress;
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  const SSL *ssl = nullptr;
#endif

  bool has_header(const std::string &key) const;
  std::string get_header_value(const std::string &key, size_t id = 0) const;
  template <typename T>
  T get_header_value(const std::string &key, size_t id = 0) const;
  size_t get_header_value_count(const std::string &key) const;
  void set_header(const std::string &key, const std::string &val);
  void set_header(const std::string &key, const char *val);

  bool has_param(const std::string &key) const;
  std::string get_param_value(const std::string &key, size_t id = 0) const;
  size_t get_param_value_count(const std::string &key) const;

  bool is_multipart_form_data() const;

  bool has_file(const std::string &key) const;
  MultipartFormData get_file_value(const std::string &key) const;
  std::vector<MultipartFormData> get_file_values(const std::string &key) const;

  // private members...
  size_t redirect_count_ = CPPHTTPLIB_REDIRECT_MAX_COUNT;
  size_t content_length_ = 0;
  ContentProvider content_provider_;
  bool is_chunked_content_provider_ = false;
  size_t authorization_count_ = 0;
};

struct Response {
  std::string version;
  int status = -1;
  std::string reason;
  Headers headers;
  std::string body;
  std::string location; // Redirect location

  bool has_header(const std::string &key) const;
  std::string get_header_value(const std::string &key, size_t id = 0) const;
  template <typename T>
  T get_header_value(const std::string &key, size_t id = 0) const;
  size_t get_header_value_count(const std::string &key) const;
  void set_header(const std::string &key, const std::string &val);
  void set_header(const std::string &key, const char *val);

  void set_redirect(const std::string &url, int status = 302);
  void set_content(const std::string &s, const std::string &content_type);
  void set_content(const char *s, size_t n, const std::string &content_type);
  void set_content(const char *s, const std::string &content_type);

  void set_content_provider(
      size_t length, const std::string &content_type, ContentProvider provider,
      ContentProviderResourceReleaser resource_releaser = nullptr);

  void set_content_provider(
      const std::string &content_type, ContentProviderWithoutLength provider,
      ContentProviderResourceReleaser resource_releaser = nullptr);

  void set_chunked_content_provider(
      const std::string &content_type, ContentProviderWithoutLength provider,
      ContentProviderResourceReleaser resource_releaser = nullptr);

  Response() = default;
  Response(const Response &) = default;
  Response &operator=(const Response &) = default;
  Response(Response &&) = default;
  Response &operator=(Response &&) = default;
  ~Response() {
    if (content_provider_resource_releaser_) {
      content_provider_resource_releaser_(false);
    }
  }

  // private members...
  size_t content_length_ = 0;
  ContentProvider content_provider_;
  ContentProviderResourceReleaser content_provider_resource_releaser_;
  bool is_chunked_content_provider_ = false;
  bool compress_ = false;
};

class Stream {
public:
  virtual ~Stream() = default;

  virtual bool is_readable() const = 0;
  virtual bool is_writable() const = 0;

  virtual ssize_t read(char *ptr, size_t size) = 0;
  virtual ssize_t write(const char *ptr, size_t size) = 0;
  virtual void get_remote_ip_and_port(std::string &ip, int &port) const = 0;
  virtual void get_local_ip_and_port(std::string &ip, int &port) const = 0;

  template <typename... Args>
  ssize_t write_format(const char *fmt, const Args &...args);
  ssize_t write(const char *ptr);
  ssize_t write(const std::string &s);
};

class TaskQueue {
public:
  TaskQueue() = default;
  virtual ~TaskQueue() = default;

  virtual bool enqueue(std::function<void()> fn) = 0;
  virtual void shutdown() = 0;

  virtual void on_idle(){};
};

class ThreadPool : public TaskQueue {
public:
  explicit ThreadPool(size_t n) : shutdown_(false) {
    while (n) {
      threads_.emplace_back(worker(*this));
      n--;
    }
  }

  ThreadPool(const ThreadPool &) = delete;
  ~ThreadPool() override = default;

  bool enqueue(std::function<void()> fn) override {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (shutdown_) { return false; }
      jobs_.push_back(std::move(fn));
    }
    cond_.notify_one();
    return true;
  }

  void shutdown() override {
    // Stop all worker threads...
    {
      std::unique_lock<std::mutex> lock(mutex_);
      shutdown_ = true;
    }
    cond_.notify_all();

    // Join...
    for (auto &t : threads_) {
      t.join();
    }
  }

private:
  struct worker {
    explicit worker(ThreadPool &pool) : pool_(pool) {}

    void operator()() {
      for (;;) {
        std::function<void()> job;
        {
          std::unique_lock<std::mutex> lock(pool_.mutex_);
          pool_.cond_.wait(
              lock, [&] { return !pool_.jobs_.empty() || pool_.shutdown_; });
          if (pool_.shutdown_ && pool_.jobs_.empty()) { break; }
          job = std::move(pool_.jobs_.front());
          pool_.jobs_.pop_front();
        }
        job();
      }
    }

    ThreadPool &pool_;
  };
  friend struct worker;

  std::vector<std::thread> threads_;
  std::list<std::function<void()>> jobs_;

  bool shutdown_;

  std::condition_variable cond_;
  std::mutex mutex_;
};

using Logger = std::function<void(const Request &, const Response &)>;

using SocketOptions = std::function<void(socket_t sock)>;

void default_socket_options(socket_t sock);

class Server {
public:
  using Handler = std::function<void(const Request &, Response &)>;

  using ExceptionHandler =
      std::function<void(const Request &, Response &, std::exception_ptr ep)>;

  using HandlerWithContentReader = std::function<void(
      const Request &, Response &, const ContentReader &content_reader)>;

  using HandlerWithResponse =
      std::function<void(const Request &, Response &, const Response &)>;

  using Expect100ContinueHandler =
      std::function<int(const Request &, Response &)>;

  Server();

  virtual ~Server();

  virtual bool is_valid() const;

  Server &Get(const std::string &pattern, Handler handler);
  Server &Post(const std::string &pattern, Handler handler);
  Server &Post(const std::string &pattern, HandlerWithContentReader handler);
  Server &Put(const std::string &pattern, Handler handler);
  Server &Put(const std::string &pattern, HandlerWithContentReader handler);
  Server &Patch(const std::string &pattern, Handler handler);
  Server &Patch(const std::string &pattern, HandlerWithContentReader handler);
  Server &Delete(const std::string &pattern, Handler handler);
  Server &Delete(const std::string &pattern, HandlerWithContentReader handler);
  Server &Options(const std::string &pattern, Handler handler);

  bool set_base_dir(const std::string &dir,
                    const std::string &mount_point = std::string());
  bool set_mount_point(const std::string &mount_point, const std::string &dir,
                       Headers headers = Headers());
  bool remove_mount_point(const std::string &mount_point);
  Server &set_file_extension_and_mimetype_mapping(const std::string &ext,
                                                   const std::string &mime);
  Server &set_file_request_handler(Handler handler);

  Server &set_error_handler(HandlerWithResponse handler);
  Server &set_error_handler(Handler handler);
  Server &set_exception_handler(ExceptionHandler handler);
  Server &set_pre_routing_handler(HandlerWithResponse handler);
  Server &set_post_routing_handler(Handler handler);

  Server &set_expect_100_continue_handler(Expect100ContinueHandler handler);
  Server &set_logger(Logger logger);

  Server &set_address_family(int family);
  Server &set_tcp_nodelay(bool on);
  Server &set_socket_options(SocketOptions socket_options);

  Server &set_default_headers(Headers headers);

  Server &set_keep_alive_max_count(size_t count);
  Server &set_keep_alive_timeout(time_t sec);

  Server &set_read_timeout(time_t sec, time_t usec = 0);
  Server &set_write_timeout(time_t sec, time_t usec = 0);
  Server &set_idle_interval(time_t sec, time_t usec = 0);

  Server &set_payload_max_length(size_t length);

  bool bind_to_port(const std::string &host, int port, int socket_flags = 0);
  int bind_to_any_port(const std::string &host, int socket_flags = 0);
  bool listen_after_bind();

  bool listen(const std::string &host, int port, int socket_flags = 0);

  bool is_running() const;
  void wait_until_ready() const;
  void stop();

  std::function<TaskQueue *(void)> new_task_queue;

protected:
  bool process_request(Stream &strm, bool close_connection,
                       bool &connection_closed,
                       const std::function<void(Request &)> &setup_request);

  std::atomic<socket_t> svr_sock_{INVALID_SOCKET};
  size_t keep_alive_max_count_ = CPPHTTPLIB_KEEPALIVE_MAX_COUNT;
  time_t keep_alive_timeout_sec_ = CPPHTTPLIB_KEEPALIVE_TIMEOUT_SECOND;
  time_t read_timeout_sec_ = CPPHTTPLIB_READ_TIMEOUT_SECOND;
  time_t read_timeout_usec_ = CPPHTTPLIB_READ_TIMEOUT_USECOND;
  time_t write_timeout_sec_ = CPPHTTPLIB_WRITE_TIMEOUT_SECOND;
  time_t write_timeout_usec_ = CPPHTTPLIB_WRITE_TIMEOUT_USECOND;
  time_t idle_interval_sec_ = CPPHTTPLIB_IDLE_INTERVAL_SECOND;
  time_t idle_interval_usec_ = CPPHTTPLIB_IDLE_INTERVAL_USECOND;
  size_t payload_max_length_ = CPPHTTPLIB_PAYLOAD_MAX_LENGTH;

private:
  using Handlers = std::vector<std::pair<std::regex, Handler>>;
  using HandlersForContentReader =
      std::vector<std::pair<std::regex, HandlerWithContentReader>>;

  socket_t create_server_socket(const std::string &host, int port,
                                int socket_flags, SocketOptions socket_options) const;
  int bind_internal(const std::string &host, int port, int socket_flags);
  bool listen_internal();

  bool routing(Request &req, Response &res, Stream &strm);
  bool handle_file_request(const Request &req, Response &res,
                           bool head = false);
  bool dispatch_request(Request &req, Response &res, const Handlers &handlers);
  bool
  dispatch_request_for_content_reader(Request &req, Response &res,
                                      ContentReader content_reader,
                                      const HandlersForContentReader &handlers);

  bool parse_request_line(const char *s, Request &req);
  void apply_ranges(const Request &req, Response &res,
                    std::string &content_type, std::string &boundary);
  bool write_response(Stream &strm, bool close_connection, const Request &req,
                      Response &res);
  bool write_response_with_content(Stream &strm, bool close_connection,
                                   const Request &req, Response &res);
  bool write_response_core(Stream &strm, bool close_connection,
                           const Request &req, Response &res,
                           bool need_apply_ranges);
  bool write_content_with_provider(Stream &strm, const Request &req,
                                   Response &res, const std::string &boundary,
                                   const std::string &content_type);
  bool read_content(Stream &strm, Request &req, Response &res);
  bool
  read_content_with_content_receiver(Stream &strm, Request &req, Response &res,
                                     ContentReceiver receiver,
                                     MultipartContentHeader multipart_header,
                                     ContentReceiver multipart_receiver);
  bool read_content_core(Stream &strm, Request &req, Response &res,
                         ContentReceiver receiver,
                         MultipartContentHeader multipart_header,
                         ContentReceiver multipart_receiver);

  virtual bool process_and_close_socket(socket_t sock);

  struct MountPointEntry {
    std::string mount_point;
    std::string base_dir;
    Headers headers;
  };
  std::vector<MountPointEntry> base_dirs_;

  std::atomic<bool> is_running_{false};
  std::atomic<bool> done_{false};

  std::map<std::string, std::string> file_extension_and_mimetype_map_;
  Handler file_request_handler_;

  Handlers get_handlers_;
  Handlers post_handlers_;
  Handlers put_handlers_;
  Handlers patch_handlers_;
  Handlers delete_handlers_;
  Handlers options_handlers_;
  HandlersForContentReader post_handlers_for_content_reader_;
  HandlersForContentReader put_handlers_for_content_reader_;
  HandlersForContentReader patch_handlers_for_content_reader_;
  HandlersForContentReader delete_handlers_for_content_reader_;

  Handler error_handler_;
  ExceptionHandler exception_handler_;
  HandlerWithResponse pre_routing_handler_;
  Handler post_routing_handler_;

  Expect100ContinueHandler expect_100_continue_handler_;

  Logger logger_;

  int address_family_ = AF_UNSPEC;
  bool tcp_nodelay_ = CPPHTTPLIB_TCP_NODELAY;
  SocketOptions socket_options_ = default_socket_options;

  Headers default_headers_;
};

enum class Error {
  Success = 0,
  Unknown,
  Connection,
  BindIPAddress,
  Read,
  Write,
  ExceedRedirectCount,
  Canceled,
  SSLConnection,
  SSLLoadingCerts,
  SSLServerVerification,
  UnsupportedMultipartBoundaryChars,
  Compression,
  ConnectionTimeout,
  ProxyConnection,
};

std::string to_string(const Error error);

std::ostream &operator<<(std::ostream &os, const Error &obj);

class Result {
public:
  Result() = default;
  Result(std::unique_ptr<Response> &&res, Error err,
         Headers &&request_headers = Headers{})
      : res_(std::move(res)), err_(err),
        request_headers_(std::move(request_headers)) {}
  // Response
  operator bool() const { return res_ != nullptr; }
  bool operator==(std::nullptr_t) const { return res_ == nullptr; }
  bool operator!=(std::nullptr_t) const { return res_ != nullptr; }
  const Response &operator*() const { return *res_; }
  const Response *operator->() const { return res_.get(); }

  // Error
  Error error() const { return err_; }

  // Request Headers
  bool has_request_header(const std::string &key) const;
  std::string get_request_header_value(const std::string &key,
                                       size_t id = 0) const;
  template <typename T>
  T get_request_header_value(const std::string &key, size_t id = 0) const;
  size_t get_request_header_value_count(const std::string &key) const;

private:
  std::unique_ptr<Response> res_;
  Error err_ = Error::Unknown;
  Headers request_headers_;
};

class Client {
public:
  // Universal interface
  explicit Client(const std::string &scheme_host_port);
  explicit Client(const std::string &scheme_host_port,
                  const std::string &client_cert_path,
                  const std::string &client_key_path);

  // HTTP only interface
  explicit Client(const std::string &host, int port);
  explicit Client(const std::string &host, int port,
                  const std::string &client_cert_path,
                  const std::string &client_key_path);

  Client(Client &&) = default;

  ~Client();

  bool is_valid() const;

  Result Get(const std::string &path);
  Result Get(const std::string &path, const Headers &headers);
  Result Get(const std::string &path, Progress progress);
  Result Get(const std::string &path, const Headers &headers,
             Progress progress);
  Result Get(const std::string &path, ContentReceiver content_receiver);
  Result Get(const std::string &path, const Headers &headers,
             ContentReceiver content_receiver);
  Result Get(const std::string &path, ContentReceiver content_receiver,
             Progress progress);
  Result Get(const std::string &path, const Headers &headers,
             ContentReceiver content_receiver, Progress progress);
  Result Get(const std::string &path, ResponseHandler response_handler,
             ContentReceiver content_receiver);
  Result Get(const std::string &path, const Headers &headers,
             ResponseHandler response_handler,
             ContentReceiver content_receiver);
  Result Get(const std::string &path, const Headers &headers,
             ResponseHandler response_handler, ContentReceiver content_receiver,
             Progress progress);
  Result Get(const std::string &path, ResponseHandler response_handler,
             ContentReceiver content_receiver, Progress progress);

  Result Head(const std::string &path);
  Result Head(const std::string &path, const Headers &headers);

  Result Post(const std::string &path);
  Result Post(const std::string &path, const Headers &headers);
  Result Post(const std::string &path, const char *body, size_t content_length,
              const std::string &content_type);
  Result Post(const std::string &path, const Headers &headers,
              const char *body, size_t content_length,
              const std::string &content_type);
  Result Post(const std::string &path, const std::string &body,
              const std::string &content_type);
  Result Post(const std::string &path, const Headers &headers,
              const std::string &body, const std::string &content_type);
  Result Post(const std::string &path, size_t content_length,
              ContentProvider content_provider,
              const std::string &content_type);
  Result Post(const std::string &path, const Headers &headers,
              size_t content_length, ContentProvider content_provider,
              const std::string &content_type);
  Result Post(const std::string &path, ContentProviderWithoutLength content_provider,
              const std::string &content_type);
  Result Post(const std::string &path, const Headers &headers,
              ContentProviderWithoutLength content_provider,
              const std::string &content_type);
  Result Post(const std::string &path, const Params &params);
  Result Post(const std::string &path, const Headers &headers,
              const Params &params);
  Result Post(const std::string &path, const MultipartFormDataItems &items);
  Result Post(const std::string &path, const Headers &headers,
              const MultipartFormDataItems &items);
  Result Post(const std::string &path, const Headers &headers,
              const MultipartFormDataItems &items, const std::string &boundary);
  Result Post(const std::string &path, const Headers &headers,
              const MultipartFormDataItems &items,
              const MultipartFormDataProviders &providers);
  Result Post(const std::string &path, const Headers &headers,
              const MultipartFormDataItems &items,
              const MultipartFormDataProviders &providers,
              const std::string &boundary);

  Result Put(const std::string &path);
  Result Put(const std::string &path, const char *body, size_t content_length,
             const std::string &content_type);
  Result Put(const std::string &path, const Headers &headers,
             const char *body, size_t content_length,
             const std::string &content_type);
  Result Put(const std::string &path, const std::string &body,
             const std::string &content_type);
  Result Put(const std::string &path, const Headers &headers,
             const std::string &body, const std::string &content_type);
  Result Put(const std::string &path, size_t content_length,
             ContentProvider content_provider, const std::string &content_type);
  Result Put(const std::string &path, const Headers &headers,
             size_t content_length, ContentProvider content_provider,
             const std::string &content_type);
  Result Put(const std::string &path, ContentProviderWithoutLength content_provider,
             const std::string &content_type);
  Result Put(const std::string &path, const Headers &headers,
             ContentProviderWithoutLength content_provider,
             const std::string &content_type);
  Result Put(const std::string &path, const Params &params);
  Result Put(const std::string &path, const Headers &headers,
             const Params &params);
  Result Put(const std::string &path, const MultipartFormDataItems &items);
  Result Put(const std::string &path, const Headers &headers,
             const MultipartFormDataItems &items);
  Result Put(const std::string &path, const Headers &headers,
             const MultipartFormDataItems &items, const std::string &boundary);
  Result Put(const std::string &path, const Headers &headers,
             const MultipartFormDataItems &items,
             const MultipartFormDataProviders &providers);
  Result Put(const std::string &path, const Headers &headers,
             const MultipartFormDataItems &items,
             const MultipartFormDataProviders &providers,
             const std::string &boundary);

  Result Patch(const std::string &path);
  Result Patch(const std::string &path, const char *body, size_t content_length,
               const std::string &content_type);
  Result Patch(const std::string &path, const Headers &headers,
               const char *body, size_t content_length,
               const std::string &content_type);
  Result Patch(const std::string &path, const std::string &body,
               const std::string &content_type);
  Result Patch(const std::string &path, const Headers &headers,
               const std::string &body, const std::string &content_type);
  Result Patch(const std::string &path, size_t content_length,
               ContentProvider content_provider,
               const std::string &content_type);
  Result Patch(const std::string &path, const Headers &headers,
               size_t content_length, ContentProvider content_provider,
               const std::string &content_type);
  Result Patch(const std::string &path, ContentProviderWithoutLength content_provider,
               const std::string &content_type);
  Result Patch(const std::string &path, const Headers &headers,
               ContentProviderWithoutLength content_provider,
               const std::string &content_type);

  Result Delete(const std::string &path);
  Result Delete(const std::string &path, const Headers &headers);
  Result Delete(const std::string &path, const char *body,
                size_t content_length, const std::string &content_type);
  Result Delete(const std::string &path, const Headers &headers,
                const char *body, size_t content_length,
                const std::string &content_type);
  Result Delete(const std::string &path, const std::string &body,
                const std::string &content_type);
  Result Delete(const std::string &path, const Headers &headers,
                const std::string &body, const std::string &content_type);

  Result Options(const std::string &path);
  Result Options(const std::string &path, const Headers &headers);

  bool send(Request &req, Response &res, Error &error);
  Result send(const Request &req);

  size_t is_socket_open() const;

  socket_t socket() const;

  void stop();

  void set_hostname_addr_map(std::map<std::string, std::string> addr_map);

  void set_default_headers(Headers headers);

  void set_address_family(int family);
  void set_tcp_nodelay(bool on);
  void set_socket_options(SocketOptions socket_options);

  void set_connection_timeout(time_t sec, time_t usec = 0);
  void set_read_timeout(time_t sec, time_t usec = 0);
  void set_write_timeout(time_t sec, time_t usec = 0);

  void set_basic_auth(const std::string &username, const std::string &password);
  void set_bearer_token_auth(const std::string &token);
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  void set_digest_auth(const std::string &username,
                       const std::string &password);
#endif

  void set_keep_alive(bool on);
  void set_follow_location(bool on);

  void set_url_encode(bool on);

  void set_compress(bool on);

  void set_decompress(bool on);

  void set_interface(const std::string &intf);

  void set_proxy(const std::string &host, int port);
  void set_proxy_basic_auth(const std::string &username,
                            const std::string &password);
  void set_proxy_bearer_token_auth(const std::string &token);
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  void set_proxy_digest_auth(const std::string &username,
                             const std::string &password);
#endif

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  void set_ca_cert_path(const std::string &ca_cert_file_path,
                        const std::string &ca_cert_dir_path = std::string());
  void set_ca_cert_store(X509_STORE *ca_cert_store);
  void enable_server_certificate_verification(bool enabled);
#endif

  void set_logger(Logger logger);

protected:
  struct Socket {
    socket_t sock = INVALID_SOCKET;
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
    SSL *ssl = nullptr;
#endif

    bool is_open() const {
      return sock != INVALID_SOCKET;
    }
  };

  Result send_(Request &&req);

  virtual bool create_and_connect_socket(Socket &socket, Error &error);

  // All of these methods have been made virtual so that
  // the client can be inherited and the behavior of these
  // methods can be overridden.
  virtual void shutdown_ssl(Socket &socket, bool shutdown_gracefully);
  virtual void shutdown_socket(Socket &socket);
  virtual void close_socket(Socket &socket);

  bool process_request(Stream &strm, Request &req, Response &res,
                       bool close_connection, Error &error);

  bool write_content_with_provider(Stream &strm, const Request &req,
                                   Error &error);

  void copy_settings(const Client &rhs);

  // Socket endpoint information
  const std::string host_;
  const int port_;
  const std::string host_and_port_;

  // Current open socket
  Socket socket_;
  mutable std::mutex socket_mutex_;
  std::recursive_mutex request_mutex_;

  // These are all protected under socket_mutex
  size_t socket_requests_in_flight_ = 0;
  std::thread::id socket_requests_are_from_thread_ = std::thread::id{};
  bool socket_should_be_closed_when_request_is_done_ = false;

  // Hostname-IP map
  std::map<std::string, std::string> addr_map_;

  // Default headers
  Headers default_headers_;

  // Settings
  std::string client_cert_path_;
  std::string client_key_path_;

  time_t connection_timeout_sec_ = CPPHTTPLIB_CONNECTION_TIMEOUT_SECOND;
  time_t connection_timeout_usec_ = CPPHTTPLIB_CONNECTION_TIMEOUT_USECOND;
  time_t read_timeout_sec_ = CPPHTTPLIB_READ_TIMEOUT_SECOND;
  time_t read_timeout_usec_ = CPPHTTPLIB_READ_TIMEOUT_USECOND;
  time_t write_timeout_sec_ = CPPHTTPLIB_WRITE_TIMEOUT_SECOND;
  time_t write_timeout_usec_ = CPPHTTPLIB_WRITE_TIMEOUT_USECOND;

  std::string basic_auth_username_;
  std::string basic_auth_password_;
  std::string bearer_token_auth_token_;
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  std::string digest_auth_username_;
  std::string digest_auth_password_;
#endif

  bool keep_alive_ = false;
  bool follow_location_ = false;

  bool url_encode_ = true;

  int address_family_ = AF_UNSPEC;
  bool tcp_nodelay_ = CPPHTTPLIB_TCP_NODELAY;
  SocketOptions socket_options_ = nullptr;

  bool compress_ = false;
  bool decompress_ = true;

  std::string interface_;

  std::string proxy_host_;
  int proxy_port_ = -1;

  std::string proxy_basic_auth_username_;
  std::string proxy_basic_auth_password_;
  std::string proxy_bearer_token_auth_token_;
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  std::string proxy_digest_auth_username_;
  std::string proxy_digest_auth_password_;
#endif

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  std::string ca_cert_file_path_;
  std::string ca_cert_dir_path_;
  X509_STORE *ca_cert_store_ = nullptr;
  bool server_certificate_verification_ = true;
#endif

  Logger logger_;

private:
  bool send_(const Request &req, Response &res, Error &error);
  Result send_(const Request &req);

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  bool is_ssl() const;
#endif
};

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
class SSLServer : public Server {
public:
  SSLServer(const char *cert_path, const char *private_key_path,
            const char *client_ca_cert_file_path = nullptr,
            const char *client_ca_cert_dir_path = nullptr,
            const char *private_key_password = nullptr);

  SSLServer(X509 *cert, EVP_PKEY *private_key,
            X509_STORE *client_ca_cert_store = nullptr);

  SSLServer(
      const std::function<bool(SSL_CTX &ssl_ctx)> &setup_ssl_ctx_callback);

  ~SSLServer() override;

  bool is_valid() const override;

  SSL_CTX *ssl_context() const;

private:
  bool process_and_close_socket(socket_t sock) override;

  SSL_CTX *ctx_;
  std::mutex ctx_mutex_;
};

class SSLClient : public Client {
public:
  explicit SSLClient(const std::string &host);
  explicit SSLClient(const std::string &host, int port);
  explicit SSLClient(const std::string &host, int port,
                     const std::string &client_cert_path,
                     const std::string &client_key_path);
  explicit SSLClient(const std::string &host, int port, X509 *client_cert,
                     EVP_PKEY *client_key);

  ~SSLClient() override;

  bool is_valid() const override;

  void set_ca_cert_store(X509_STORE *ca_cert_store);
  void set_ca_cert_path(const std::string &ca_cert_file_path,
                        const std::string &ca_cert_dir_path = std::string());
  void enable_server_certificate_verification(bool enabled);

  long get_openssl_verify_result() const;

  SSL_CTX *ssl_context() const;

private:
  bool create_and_connect_socket(Socket &socket, Error &error) override;
  void shutdown_ssl(Socket &socket, bool shutdown_gracefully) override;

  bool process_socket(const Socket &socket,
                      std::function<bool(Stream &strm)> callback);
  bool is_ssl() const override;

  bool connect_with_proxy(Socket &socket, Response &res, bool &success,
                          Error &error);
  bool initialize_ssl(Socket &socket, Error &error);

  bool load_certs();

  bool verify_host(X509 *server_cert) const;
  bool verify_host_with_subject_alt_name(X509 *server_cert) const;
  bool verify_host_with_common_name(X509 *server_cert) const;
  bool check_host_name(const char *pattern, size_t pattern_len) const;

  SSL_CTX *ctx_;
  std::mutex ctx_mutex_;
  std::once_flag initialize_cert_;

  std::vector<std::string> host_components_;

  long verify_result_ = 0;

  friend class ClientImpl;
};
#endif

// Implementation...

namespace detail {

template <class T, class U, class V>
inline void split(const T &str, const U &delimiter, V fn) {
  size_t offset = 0;
  while (true) {
    auto pos = str.find(delimiter, offset);
    if (pos == T::npos) {
      fn(offset, T::npos);
      break;
    }
    fn(offset, pos);
    offset = pos + 1;
  }
}

template <class T>
inline T trim_copy(const T &s) {
  auto r = s;
  r.erase(r.begin(),
          std::find_if(r.begin(), r.end(),
                       [](typename T::value_type ch) { return !std::isspace(ch); }));
  r.erase(std::find_if(r.rbegin(), r.rend(),
                       [](typename T::value_type ch) { return !std::isspace(ch); })
              .base(),
          r.end());
  return r;
}

} // namespace detail

// Implementation continues...

inline void Server::set_base_dir(const std::string &dir,
                                 const std::string &mount_point) {
  set_mount_point(mount_point, dir);
}

inline bool Server::set_mount_point(const std::string &mount_point,
                                    const std::string &dir, Headers headers) {
  if (detail::is_dir(dir)) {
    std::string mnt = !mount_point.empty() ? mount_point : "/";
    if (!mnt.empty() && mnt[0] == '/') {
      base_dirs_.push_back({mnt, dir, std::move(headers)});
      return true;
    }
  }
  return false;
}

inline bool Server::remove_mount_point(const std::string &mount_point) {
  for (auto it = base_dirs_.begin(); it != base_dirs_.end(); ++it) {
    if (it->mount_point == mount_point) {
      base_dirs_.erase(it);
      return true;
    }
  }
  return false;
}

inline Server &
Server::set_file_extension_and_mimetype_mapping(const std::string &ext,
                                                 const std::string &mime) {
  file_extension_and_mimetype_map_[ext] = mime;
  return *this;
}

inline Server &Server::set_file_request_handler(Handler handler) {
  file_request_handler_ = std::move(handler);
  return *this;
}

inline Server &Server::set_error_handler(HandlerWithResponse handler) {
  error_handler_ = [handler](const Request &req, Response &res) {
    handler(req, res, Response{});
  };
  return *this;
}

inline Server &Server::set_error_handler(Handler handler) {
  error_handler_ = std::move(handler);
  return *this;
}

inline Server &Server::set_exception_handler(ExceptionHandler handler) {
  exception_handler_ = std::move(handler);
  return *this;
}

inline Server &Server::set_pre_routing_handler(HandlerWithResponse handler) {
  pre_routing_handler_ = std::move(handler);
  return *this;
}

inline Server &Server::set_post_routing_handler(Handler handler) {
  post_routing_handler_ = std::move(handler);
  return *this;
}

inline Server &Server::set_logger(Logger logger) {
  logger_ = std::move(logger);
  return *this;
}

inline Server &
Server::set_expect_100_continue_handler(Expect100ContinueHandler handler) {
  expect_100_continue_handler_ = std::move(handler);
  return *this;
}

inline Server &Server::set_address_family(int family) {
  address_family_ = family;
  return *this;
}

inline Server &Server::set_tcp_nodelay(bool on) {
  tcp_nodelay_ = on;
  return *this;
}

inline Server &Server::set_socket_options(SocketOptions socket_options) {
  socket_options_ = std::move(socket_options);
  return *this;
}

inline Server &Server::set_default_headers(Headers headers) {
  default_headers_ = std::move(headers);
  return *this;
}

inline Server &Server::set_keep_alive_max_count(size_t count) {
  keep_alive_max_count_ = count;
  return *this;
}

inline Server &Server::set_keep_alive_timeout(time_t sec) {
  keep_alive_timeout_sec_ = sec;
  return *this;
}

inline Server &Server::set_read_timeout(time_t sec, time_t usec) {
  read_timeout_sec_ = sec;
  read_timeout_usec_ = usec;
  return *this;
}

inline Server &Server::set_write_timeout(time_t sec, time_t usec) {
  write_timeout_sec_ = sec;
  write_timeout_usec_ = usec;
  return *this;
}

inline Server &Server::set_idle_interval(time_t sec, time_t usec) {
  idle_interval_sec_ = sec;
  idle_interval_usec_ = usec;
  return *this;
}

inline Server &Server::set_payload_max_length(size_t length) {
  payload_max_length_ = length;
  return *this;
}

inline bool Server::bind_to_port(const std::string &host, int port,
                                 int socket_flags) {
  return bind_internal(host, port, socket_flags) >= 0;
}
inline int Server::bind_to_any_port(const std::string &host,
                                    int socket_flags) {
  return bind_internal(host, 0, socket_flags);
}

inline bool Server::listen_after_bind() { return listen_internal(); }

inline bool Server::listen(const std::string &host, int port,
                           int socket_flags) {
  return bind_to_port(host, port, socket_flags) && listen_after_bind();
}

inline bool Server::is_running() const { return is_running_; }

inline void Server::wait_until_ready() const {
  while (!is_running() && !done_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

inline void Server::stop() {
  if (is_running_) {
    assert(svr_sock_ != INVALID_SOCKET);
    std::atomic<socket_t> sock(svr_sock_.exchange(INVALID_SOCKET));
    detail::shutdown_socket(sock);
    detail::close_socket(sock);
  }
}

inline Server &Server::Get(const std::string &pattern, Handler handler) {
  get_handlers_.push_back(
      std::make_pair(std::regex(pattern), std::move(handler)));
  return *this;
}

inline Server &Server::Post(const std::string &pattern, Handler handler) {
  post_handlers_.push_back(
      std::make_pair(std::regex(pattern), std::move(handler)));
  return *this;
}

inline Server &Server::Post(const std::string &pattern,
                            HandlerWithContentReader handler) {
  post_handlers_for_content_reader_.push_back(
      std::make_pair(std::regex(pattern), std::move(handler)));
  return *this;
}

inline Server &Server::Put(const std::string &pattern, Handler handler) {
  put_handlers_.push_back(
      std::make_pair(std::regex(pattern), std::move(handler)));
  return *this;
}

inline Server &Server::Put(const std::string &pattern,
                           HandlerWithContentReader handler) {
  put_handlers_for_content_reader_.push_back(
      std::make_pair(std::regex(pattern), std::move(handler)));
  return *this;
}

inline Server &Server::Patch(const std::string &pattern, Handler handler) {
  patch_handlers_.push_back(
      std::make_pair(std::regex(pattern), std::move(handler)));
  return *this;
}

inline Server &Server::Patch(const std::string &pattern,
                             HandlerWithContentReader handler) {
  patch_handlers_for_content_reader_.push_back(
      std::make_pair(std::regex(pattern), std::move(handler)));
  return *this;
}

inline Server &Server::Delete(const std::string &pattern, Handler handler) {
  delete_handlers_.push_back(
      std::make_pair(std::regex(pattern), std::move(handler)));
  return *this;
}

inline Server &Server::Delete(const std::string &pattern,
                              HandlerWithContentReader handler) {
  delete_handlers_for_content_reader_.push_back(
      std::make_pair(std::regex(pattern), std::move(handler)));
  return *this;
}

inline Server &Server::Options(const std::string &pattern, Handler handler) {
  options_handlers_.push_back(
      std::make_pair(std::regex(pattern), std::move(handler)));
  return *this;
}

// Minimal implementation stubs for compilation
inline bool Request::has_header(const std::string &key) const { return false; }
inline std::string Request::get_header_value(const std::string &key, size_t id) const { return ""; }
inline void Request::set_header(const std::string &key, const std::string &val) {}
inline void Request::set_header(const std::string &key, const char *val) {}
inline bool Request::has_param(const std::string &key) const { return false; }
inline std::string Request::get_param_value(const std::string &key, size_t id) const { return ""; }

inline bool Response::has_header(const std::string &key) const { return false; }
inline std::string Response::get_header_value(const std::string &key, size_t id) const { return ""; }
inline void Response::set_header(const std::string &key, const std::string &val) {}
inline void Response::set_header(const std::string &key, const char *val) {}
inline void Response::set_content(const std::string &s, const std::string &content_type) {
  body = s;
  set_header("Content-Type", content_type);
}

inline Server::Server() = default;
inline Server::~Server() = default;
inline bool Server::is_valid() const { return true; }

inline void default_socket_options(socket_t sock) {}

} // namespace httplib

#endif // CPPHTTPLIB_HTTPLIB_H