#pragma once
#ifndef BVH_TREE_STATUS_H
#define BVH_TREE_STATUS_H

#include <string>

namespace bvh {
class Status {
  enum Code { kOK = 0, kNOT_FOUND, kERROR, kNOT_SUPPORT };

 public:
  static inline Status OK() {
    static Status ok;
    return ok;
  }

  static inline Status NOT_FOUND() {
    static Status not_found(Code::kNOT_FOUND);
    return not_found;
  }

  static inline Status ERROR() {
    static Status error(Code::kERROR);
    return error;
  }

  static inline Status NOT_SUPPORT() {
    static Status not_support(Code::kNOT_SUPPORT);
    return not_support;
  }

  static inline Status MakeOK() {
    static Status ok;
    return ok;
  }

  static inline Status MakeNotFound() { return Status(Code::kNOT_FOUND); }
  static inline Status MakeNotFound(std::string&& msg) {
    return Status(Code::kNOT_FOUND, msg);
  }
  static inline Status MakeNotFound(const std::string& msg) {
    return Status(Code::kNOT_FOUND, msg);
  }

  static inline Status MakeError(std::string&& msg) {
    return Status(Code::kERROR, msg);
  }
  static inline Status MakeError(const std::string& msg) {
    return Status(Code::kERROR, msg);
  }

  static inline Status MakeNotSupport() {
    return Status(Code::kNOT_SUPPORT, "not support, now");
  }

  Status(Code code, const std::string& msg)
      : code_(code), msg_(std::move(msg)) {}
  Status(Code code, std::string&& msg) : code_(code), msg_(std::move(msg)) {}
  Status(Code code) : code_(code) {}
  Status() : code_(Code::kOK) {}

  std::string GetMsg() { return msg_; };
  ~Status(){};

  bool operator==(const Status& o) { return code_ == o.code_; }
  bool operator!=(const Status& o) { return code_ != o.code_; }

 private:
  Code code_;
  std::string msg_;
};
}  // namespace bvh

#endif
