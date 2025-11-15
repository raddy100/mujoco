#pragma once
/**
 * @file Result.h
 * @brief Lightweight structured result type replacing legacy int/bool status codes.
 *
 * Operations that can fail return Result rather than raw bool or int status.
 * Construction-time hard failures may still throw exceptions; all other recoverable
 * issues are reported via Status and a diagnostic message.
 *
 * Thread-safety: Result and Status are trivially thread-safe; the message string is
 * immutable once the Result object is constructed.
 */

#include <string>
#include <string_view>

/**
 * @brief Enumeration of operation status codes.
 *
 * Semantics:
 * - Ok: Operation succeeded.
 * - NotReady: Required dependencies (model/spec/data, or chain preconditions) missing.
 * - IOError: File or stream error (open/read/write).
 * - InvalidArgument: Provided argument failed validation.
 * - RecompileFailed: MuJoCo recompile step failed.
 * - InternalError: Unexpected invariant violation or other logic failure.
 *
 * Usage Guidelines:
 *  - Prefer propagating Status via Result; avoid throwing for recoverable failures.
 *  - Functions returning Result should never throw after returning Status::Ok.
 */
enum class Status {
  Ok,             ///< Operation succeeded.
  NotReady,       ///< Dependencies missing or not initialized.
  IOError,        ///< File or stream I/O failure.
  InvalidArgument,///< Provided argument failed validation checks.
  RecompileFailed,///< MuJoCo mj_recompile failure.
  InternalError   ///< Unexpected internal invariant violation.
};

/**
 * @brief Structured result of an operation (value type).
 *
 * Design:
 *  - status encodes coarse outcome category.
 *  - message provides optional human-readable diagnostic (empty on success by convention).
 *  - Trivially copyable / movable; safe to return by value.
 *
 * Invariants:
 *  - status always a valid enumerator.
 *  - message may be empty; consumers should not rely on non-empty for failure.
 */
struct Result {
  Status status{Status::Ok};       ///< Status code of the operation.
  std::string message{};           ///< Optional diagnostic message (empty if not required).

  /// Predicate indicating success (status == Status::Ok).
  [[nodiscard]] bool ok() const noexcept { return status == Status::Ok; }

  // Factory helpers (prefer over direct construction for clarity).
  static Result success() noexcept { return Result{Status::Ok, {}}; }
  static Result notReady(std::string_view msg) { return Result{Status::NotReady, std::string(msg)}; }
  static Result ioError(std::string_view msg) { return Result{Status::IOError, std::string(msg)}; }
  static Result invalidArg(std::string_view msg) { return Result{Status::InvalidArgument, std::string(msg)}; }
  static Result recompileFailed(std::string_view msg) { return Result{Status::RecompileFailed, std::string(msg)}; }
  static Result internalError(std::string_view msg) { return Result{Status::InternalError, std::string(msg)}; }
};

/**
 * @brief Convert Status to short string literal (for logging/debug).
 * @param s Status enumerator.
 * @return Null-terminated constant string representation.
 */
inline const char* toString(Status s) noexcept {
  switch (s) {
    case Status::Ok: return "Ok";
    case Status::NotReady: return "NotReady";
    case Status::IOError: return "IOError";
    case Status::InvalidArgument: return "InvalidArgument";
    case Status::RecompileFailed: return "RecompileFailed";
    case Status::InternalError: return "InternalError";
    default: return "Unknown";
  }
}