# C++ 代码编写规范

## 适用范围与优先级

- 本规范适用于本仓库新增的、由项目维护的 C++ 代码，包括头文件和源文件。
- 基础规范为 [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)。下文明确列出的规则优先于 Google 默认规则；未说明的部分遵循 Google 规范。
- 修改现有代码时，新增代码和本次修改涉及的代码应遵循本规范，不为统一格式而批量重排无关历史代码或重命名无关接口。
- 第三方依赖、外部导入代码和自动生成代码不纳入格式修改；如需调整生成代码，应优先修改生成器或模板。
- 语言标准、编译选项及依赖版本以项目构建配置为准，不因引用 Google 规范而自行升级。

## 缩进与对齐

- 使用真正的 Tab 字符进行缩进，不以空格模拟 Tab。
- 每一级缩进为 8 列，Tab 制表位宽度为 8 列。
- 使用空格进行对齐，例如续行参数与前一行参数的对齐；不要使用 Tab 进行对齐。
- 不在行末保留多余空白。

## 行长度

- 每行最多 140 列，Tab 按 8 列制表位展开计算。
- 超长表达式、参数列表和注释应合理换行；字符串可在不改变内容的前提下使用相邻字符串字面量拆分。
- 不得为满足行长限制而改变字符串内容、程序行为或接口语义。确实无法拆分的内容应在代码审查中说明原因。

## 文件扩展名

- 新增 C++ 源文件使用 `.cpp`，不使用 `.cc`。
- 新增普通 C++ 头文件遵循 Google 规范，使用 `.h`；与现有生成器或外部工具集成时遵循其要求。
- 不仅为扩展名统一而重命名已有文件；需要重命名时，同步更新构建配置及所有引用。

## 函数、方法与类命名

- 普通函数和方法使用 `lowerCamelCase`，例如 `computeVelocity()`、`resetState()`。
- 类名使用 `UpperCamelCase`，例如 `FlightController`。
- 构造函数保持与类名一致，析构函数遵循 C++ 语法。
- 覆写基类方法、实现外部接口或遵循语言及库协议时，保留其要求的名称，不为命名统一而破坏兼容性。

## 私有成员变量命名

- 私有成员变量使用 `_underscore_prefixed_snake_case`，例如 `_frame_count`、`_target_velocity`。
- 不采用 Google 默认的下划线后缀形式，例如 `frame_count_`。
- 此规则仅针对私有成员变量，不推广到全局变量、命名空间名称或普通局部变量。
- 避免保留标识符：不使用双下划线，也不使用下划线后紧接大写字母的名称。

## 类访问修饰符

- `public:`、`private:` 和 `protected:` 与所属 `class` 或 `struct` 声明处于同一缩进层级，不增加额外的空格或 Tab。
- 对于顶层类，访问修饰符从行首开始；嵌套类仍保留其所在作用域的必要缩进。
- 访问修饰符下的成员声明缩进一级，即一个 Tab。

以下示例中，成员声明行以一个真实 Tab 字符缩进：

```cpp
class FlightController {
public:
	FlightController();
	void resetState();
	double computeVelocity(double elapsed_time) const;

private:
	int _frame_count = 0;
	double _target_velocity = 0.0;
};
```

## 格式化与提交前检查

- 自动格式化采用 `clang-format`，基础风格为 `Google`，核心配置应与下列内容一致：

```yaml
BasedOnStyle: Google
Language: Cpp
UseTab: ForIndentation
IndentWidth: 8
TabWidth: 8
ContinuationIndentWidth: 8
ConstructorInitializerIndentWidth: 8
ColumnLimit: 140
AccessModifierOffset: -8
```

- 格式化前确认目标文件实际使用的 `.clang-format`；子目录配置可能覆盖根目录配置。若现有配置与本规范冲突，应先明确并修正配置，不能将冲突的格式化结果视为合规。
- 对新增文件执行完整格式检查；对历史文件优先检查和格式化本次修改范围，避免产生无关差异。
- `clang-format` 不检查命名和文件扩展名，也不保证所有不可拆分内容都满足行长要求；需另行检查函数命名、私有成员命名、扩展名及超长行。可使用 `clang-tidy` 的 `readability-identifier-naming` 辅助检查命名。
- 提交前检查差异，确认无空格缩进替代 Tab、Tab 对齐、行末空白或无关格式修改，并执行与代码变更相适应的构建或测试。
- 检查工具不可用或检查未完成时，应明确说明，不得声称已通过检查。
