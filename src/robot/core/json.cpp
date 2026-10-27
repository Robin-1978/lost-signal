#include "robot/core/json.hpp"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace lost_signal::robot::core {

const Json* Json::find(const std::string& key) const {
  if (!is_object()) return nullptr;
 for (const auto& entry : as_object()) {
 if (entry.first == key) return &entry.second;
  }
  return nullptr;
}

namespace {

void dump_into(const Json& value, std::string& out) {
  if (value.is_null()) {
    out += "null";
  } else if (value.is_bool()) {
    out += value.as_bool() ? "true" : "false";
  } else if (value.is_num()) {
    char buf[32];
    // %.17g round-trips every double; shorter forms do not.
    std::snprintf(buf, sizeof(buf), "%.17g", value.as_num());
    out += buf;
  } else if (value.is_str()) {
    out += '"';
    for (char c : value.as_str()) {
      switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
          if (static_cast<unsigned char>(c) < 0x20) {
            char buf[8];
            std::snprintf(buf, sizeof(buf), "\\u%04x", c);
            out += buf;
          } else {
            out += c;
          }
      }
    }
    out += '"';
  } else if (value.is_array()) {
    out += '[';
    bool first = true;
    for (const auto& item : value.as_array()) {
      if (!first) out += ',';
      first = false;
      dump_into(item, out);
    }
    out += ']';
  } else {
    out += '{';
    bool first = true;
    for (const auto& [k, v] : value.as_object()) {
      if (!first) out += ',';
      first = false;
      dump_into(Json::Str(k), out);
      out += ':';
      dump_into(v, out);
    }
    out += '}';
  }
}

struct Parser {
  const char* p;
  const char* end;
  std::string error;

  explicit Parser(const std::string& text)
      : p(text.data()), end(text.data() + text.size()) {}

  void skip_ws() {
    while (p < end && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) ++p;
  }
  bool expect(char c) {
    skip_ws();
    if (p < end && *p == c) {
      ++p;
      return true;
    }
    return false;
  }

  std::unique_ptr<Json> parse_value() {
    skip_ws();
    if (p >= end) {
      error = "unexpected end of input";
      return nullptr;
    }
    if (*p == '{') return parse_object();
    if (*p == '[') return parse_array();
    if (*p == '"') {
      std::string s;
      if (!parse_string(s)) return nullptr;
      return std::make_unique<Json>(Json::Str(std::move(s)));
    }
    if (*p == 't' || *p == 'f' || *p == 'n') return parse_literal();
    if (*p == '-' || (*p >= '0' && *p <= '9')) return parse_number();
    error = std::string("unexpected character '") + *p + "'";
    return nullptr;
  }

  bool parse_string(std::string& out) {
    // *p == '"'
    ++p;
    while (p < end && *p != '"') {
      if (*p == '\\') {
        ++p;
        if (p >= end) break;
        switch (*p) {
          case '"': out += '"'; break;
          case '\\': out += '\\'; break;
          case '/': out += '/'; break;
          case 'n': out += '\n'; break;
          case 'r': out += '\r'; break;
          case 't': out += '\t'; break;
          case 'u': {
            if (end - p < 5) {
              error = "truncated unicode escape";
              return false;
            }
            unsigned code = 0;
            for (int i = 1; i <= 4; ++i) {
              char c = p[i];
              code <<= 4;
              if (c >= '0' && c <= '9') code |= c - '0';
              else if (c >= 'a' && c <= 'f') code |= c - 'a' + 10;
              else if (c >= 'A' && c <= 'F') code |= c - 'A' + 10;
              else {
                error = "invalid unicode escape";
                return false;
              }
            }
            // Basic plane only; sufficient for our ASCII-dominated logs.
            if (code < 0x80) {
              out += static_cast<char>(code);
            } else if (code < 0x800) {
              out += static_cast<char>(0xC0 | (code >> 6));
              out += static_cast<char>(0x80 | (code & 0x3F));
            } else {
              out += static_cast<char>(0xE0 | (code >> 12));
              out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
              out += static_cast<char>(0x80 | (code & 0x3F));
            }
            p += 5;
            continue;
          }
          default:
            error = "invalid escape";
            return false;
        }
        ++p;
      } else {
        out += *p;
        ++p;
      }
    }
    if (p >= end || *p != '"') {
      error = "unterminated string";
      return false;
    }
    ++p;
    return true;
  }

  std::unique_ptr<Json> parse_literal() {
    auto match = [&](const char* word) {
      for (const char* q = word; *q; ++q, ++p) {
        if (p >= end || *p != *q) return false;
      }
      return true;
    };
    const char* save = p;
    if (match("true")) return std::make_unique<Json>(Json::Bool(true));
    p = save;
    if (match("false")) return std::make_unique<Json>(Json::Bool(false));
    p = save;
    if (match("null")) return std::make_unique<Json>(Json::Null());
    error = "invalid literal";
    return nullptr;
  }

  std::unique_ptr<Json> parse_number() {
    const char* start = p;
    if (p < end && *p == '-') ++p;
    if (p >= end) {
      error = "truncated number";
      return nullptr;
    }
    // Reject non-standard extensions (hex, inf, nan): LSJSON numbers only.
    if (!std::isdigit(static_cast<unsigned char>(*p))) {
      error = "invalid number";
      return nullptr;
    }
    while (p < end && std::isdigit(static_cast<unsigned char>(*p))) ++p;
    if (p < end && *p == '.') {
      ++p;
      if (p >= end || !std::isdigit(static_cast<unsigned char>(*p))) {
        error = "invalid fraction";
        return nullptr;
      }
      while (p < end && std::isdigit(static_cast<unsigned char>(*p))) ++p;
    }
    if (p < end && (*p == 'e' || *p == 'E')) {
      ++p;
      if (p < end && (*p == '+' || *p == '-')) ++p;
      if (p >= end || !std::isdigit(static_cast<unsigned char>(*p))) {
        error = "invalid exponent";
        return nullptr;
      }
      while (p < end && std::isdigit(static_cast<unsigned char>(*p))) ++p;
    }
    try {
      double v = std::stod(std::string(start, p));
      if (!std::isfinite(v)) {
        error = "non-finite number";
        return nullptr;
      }
      return std::make_unique<Json>(Json::Num(v));
    } catch (const std::exception&) {
      error = "invalid number";
      return nullptr;
    }
  }

  std::unique_ptr<Json> parse_array() {
    ++p;  // [
    Json::Array items;
    skip_ws();
    if (p < end && *p == ']') {
      ++p;
      return std::make_unique<Json>(Json::Arr(std::move(items)));
    }
    while (true) {
      auto item = parse_value();
      if (!item) return nullptr;
      items.push_back(std::move(*item));
      skip_ws();
      if (p < end && *p == ',') {
        ++p;
        continue;
      }
      if (p < end && *p == ']') {
        ++p;
        return std::make_unique<Json>(Json::Arr(std::move(items)));
      }
      error = "expected ',' or ']' in array";
      return nullptr;
    }
  }

  std::unique_ptr<Json> parse_object() {
    ++p;  // {
    Json::Object members;
    skip_ws();
    if (p < end && *p == '}') {
      ++p;
      return std::make_unique<Json>(Json::Obj(std::move(members)));
    }
    while (true) {
      skip_ws();
      if (p >= end || *p != '"') {
        error = "expected string key in object";
        return nullptr;
      }
      std::string key;
      if (!parse_string(key)) return nullptr;
      if (!expect(':')) {
        error = "expected ':' in object";
        return nullptr;
      }
      auto val = parse_value();
      if (!val) return nullptr;
      members.emplace_back(std::move(key), std::move(*val));
      skip_ws();
      if (p < end && *p == ',') {
        ++p;
        continue;
      }
      if (p < end && *p == '}') {
        ++p;
        return std::make_unique<Json>(Json::Obj(std::move(members)));
      }
      error = "expected ',' or '}' in object";
      return nullptr;
    }
  }
};

}  // namespace

std::string json_dump(const Json& value) {
  std::string out;
  dump_into(value, out);
  return out;
}

std::unique_ptr<Json> json_parse(const std::string& text, std::string* error) {
  Parser parser(text);
  auto root = parser.parse_value();
  if (!root) {
    if (error) *error = parser.error;
    return nullptr;
  }
  parser.skip_ws();
  if (parser.p != parser.end) {
    if (error) *error = "trailing characters after document";
    return nullptr;
  }
  return root;
}

}  // namespace lost_signal::robot::core
