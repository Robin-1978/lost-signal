#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <variant>
#include <vector>

namespace lost_signal::robot::core {

// Minimal JSON value: exactly the subset the recorder emits and reads.
// No third-party dependency; the parser rejects anything outside the
// subset (objects, arrays, strings with escapes, numbers, true/false/null).
struct Json {
  using Array = std::vector<Json>;
  using Object = std::vector<std::pair<std::string, Json>>;
  using Value = std::variant<std::nullptr_t, bool, double, std::string, Array,
                             Object>;

  Value value = nullptr;

  static Json Null() { return Json{}; }
  static Json Bool(bool b) {
    Json j;
    j.value = b;
    return j;
  }
  static Json Num(double d) {
    Json j;
    j.value = d;
    return j;
  }
  static Json Str(std::string s) {
    Json j;
    j.value = std::move(s);
    return j;
  }
  static Json Arr(Array a) {
    Json j;
    j.value = std::move(a);
    return j;
  }
  static Json Obj(Object o) {
    Json j;
    j.value = std::move(o);
    return j;
  }

  bool is_null() const { return std::holds_alternative<std::nullptr_t>(value); }
  bool is_bool() const { return std::holds_alternative<bool>(value); }
  bool is_num() const { return std::holds_alternative<double>(value); }
  bool is_str() const { return std::holds_alternative<std::string>(value); }
  bool is_array() const { return std::holds_alternative<Array>(value); }
  bool is_object() const { return std::holds_alternative<Object>(value); }

  bool as_bool() const { return std::get<bool>(value); }
  double as_num() const { return std::get<double>(value); }
  const std::string& as_str() const { return std::get<std::string>(value); }
  const Array& as_array() const { return std::get<Array>(value); }
  const Object& as_object() const { return std::get<Object>(value); }

  // Object lookup by key; returns nullptr when absent or not an object.
  const Json* find(const std::string& key) const;
};

// Canonical serialization (stable key order = insertion order).
std::string json_dump(const Json& value);

// Parse one complete JSON document. Returns nullptr on any error;
// error text goes to `*error` when non-null.
std::unique_ptr<Json> json_parse(const std::string& text, std::string* error);

}  // namespace lost_signal::robot::core
