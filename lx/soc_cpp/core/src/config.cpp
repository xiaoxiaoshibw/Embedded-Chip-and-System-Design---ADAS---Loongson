#include "soc/config.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <functional>
#include <sstream>
#include <stdexcept>

namespace soc::config {

namespace {

// 把字符串两端空白 + 包裹的单/双引号剥掉，对应 Python `v.strip().strip('"\'')`。
std::string trimAndUnquote(std::string s) {
    auto notSpace = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), notSpace));
    s.erase(std::find_if(s.rbegin(), s.rend(), notSpace).base(), s.end());
    if (s.size() >= 2 && (s.front() == '"' || s.front() == '\'') && s.back() == s.front()) {
        s = s.substr(1, s.size() - 2);
    }
    return s;
}

// 极简扁平解析：每行 `KEY: value` 或 `KEY = value`，`#` 起为注释。
// 对应 Python `_parse_params_text`（PyYAML 不可用时的兜底路径；C++ 版本
// 直接只实现这一档，不引入 yaml-cpp 依赖——on-vehicle 部署零第三方依赖优先）。
std::unordered_map<std::string, std::string> parseParamsText(const std::string& text) {
    std::unordered_map<std::string, std::string> out;
    std::istringstream stream(text);
    std::string rawLine;
    while (std::getline(stream, rawLine)) {
        auto line = rawLine.substr(0, rawLine.find('#'));
        // trim
        auto notSpace = [](unsigned char c) { return !std::isspace(c); };
        line.erase(line.begin(), std::find_if(line.begin(), line.end(), notSpace));
        line.erase(std::find_if(line.rbegin(), line.rend(), notSpace).base(), line.end());
        if (line.empty()) continue;

        std::string::size_type sep = line.find(':');
        char sepChar = ':';
        if (sep == std::string::npos) {
            sep = line.find('=');
            sepChar = '=';
        }
        if (sep == std::string::npos) continue;
        (void)sepChar;

        std::string key = line.substr(0, sep);
        std::string value = line.substr(sep + 1);
        key.erase(key.begin(), std::find_if(key.begin(), key.end(), notSpace));
        key.erase(std::find_if(key.rbegin(), key.rend(), notSpace).base(), key.end());
        value = trimAndUnquote(value);
        if (!key.empty()) out[key] = value;
    }
    return out;
}

bool coerceBool(const std::string& raw, bool& out) {
    std::string s = raw;
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
    if (s == "1" || s == "true" || s == "yes" || s == "on") {
        out = true;
        return true;
    }
    if (s == "0" || s == "false" || s == "no" || s == "off") {
        out = false;
        return true;
    }
    return false;
}

bool coerceInt(const std::string& raw, int& out) {
    try {
        size_t pos = 0;
        double v = std::stod(raw, &pos);
        if (pos == 0) return false;
        out = static_cast<int>(v);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool coerceDouble(const std::string& raw, double& out) {
    try {
        size_t pos = 0;
        out = std::stod(raw, &pos);
        return pos != 0;
    } catch (const std::exception&) {
        return false;
    }
}

// 覆盖表：name -> (尝试应用, 若成功则记录 old/new 字符串表示)。
// 用同一份 X-Macro 列表生成，保证与 config.hpp 中的变量声明永不失配。
using Setter = std::function<bool(const std::string& raw, std::string& oldStr, std::string& newStr)>;

std::unordered_map<std::string, Setter>& registry() {
    static std::unordered_map<std::string, Setter> reg = [] {
        std::unordered_map<std::string, Setter> r;

#define SOC_CONFIG_REGISTER_STRING(NAME, VAL)                                  \
    r[#NAME] = [](const std::string& raw, std::string& oldStr, std::string& newStr) { \
        oldStr = NAME;                                                          \
        NAME = raw;                                                             \
        newStr = NAME;                                                          \
        return true;                                                            \
    };
#define SOC_CONFIG_REGISTER_BOOL(NAME, VAL)                                    \
    r[#NAME] = [](const std::string& raw, std::string& oldStr, std::string& newStr) { \
        bool v;                                                                 \
        if (!coerceBool(raw, v)) return false;                                  \
        oldStr = NAME ? "true" : "false";                                       \
        NAME = v;                                                               \
        newStr = NAME ? "true" : "false";                                       \
        return true;                                                            \
    };
#define SOC_CONFIG_REGISTER_INT(NAME, VAL)                                     \
    r[#NAME] = [](const std::string& raw, std::string& oldStr, std::string& newStr) { \
        int v;                                                                  \
        if (!coerceInt(raw, v)) return false;                                   \
        oldStr = std::to_string(NAME);                                          \
        NAME = v;                                                               \
        newStr = std::to_string(NAME);                                          \
        return true;                                                            \
    };
#define SOC_CONFIG_REGISTER_DOUBLE(NAME, VAL)                                  \
    r[#NAME] = [](const std::string& raw, std::string& oldStr, std::string& newStr) { \
        double v;                                                               \
        if (!coerceDouble(raw, v)) return false;                                \
        oldStr = std::to_string(NAME);                                          \
        NAME = v;                                                               \
        newStr = std::to_string(NAME);                                         \
        return true;                                                            \
    };

        SOC_CONFIG_STRINGS(SOC_CONFIG_REGISTER_STRING)
        SOC_CONFIG_BOOLS(SOC_CONFIG_REGISTER_BOOL)
        SOC_CONFIG_INTS(SOC_CONFIG_REGISTER_INT)
        SOC_CONFIG_DOUBLES(SOC_CONFIG_REGISTER_DOUBLE)

#undef SOC_CONFIG_REGISTER_STRING
#undef SOC_CONFIG_REGISTER_BOOL
#undef SOC_CONFIG_REGISTER_INT
#undef SOC_CONFIG_REGISTER_DOUBLE

        return r;
    }();
    return reg;
}

}  // namespace

std::vector<ParamOverride> applyParamOverrides(const std::string& params_yaml_path) {
    std::vector<ParamOverride> applied;

    std::ifstream file(params_yaml_path);
    if (!file.is_open()) {
        return applied;  // 文件不存在不是错误，对应 Python `if not os.path.exists(path): return`
    }
    std::ostringstream buf;
    buf << file.rdbuf();
    const auto data = parseParamsText(buf.str());

    auto& reg = registry();
    for (const auto& [key, raw] : data) {
        // 只接受全大写名——registry() 本身只登记全大写常量名，这里额外校验
        // 输入本身也是全大写，避免大小写不敏感匹配带来的隐式覆盖。
        bool isUpper = !key.empty() &&
                       std::all_of(key.begin(), key.end(), [](unsigned char c) {
                           return std::isupper(c) || c == '_' || std::isdigit(c);
                       });
        if (!isUpper) continue;

        auto it = reg.find(key);
        if (it == reg.end()) {
            // 未知名，或是安全关键参数（从未登记）—— 两者都静默跳过，
            // 对应 Python 里 "ignore unknown/forbidden param" 分支。
            continue;
        }
        std::string oldStr, newStr;
        if (it->second(raw, oldStr, newStr)) {
            applied.push_back(ParamOverride{key, oldStr, newStr});
        }
    }
    return applied;
}

}  // namespace soc::config
