//======================================================================================================================
//  C O P Y R I G H T
//----------------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 by Robert Bosch GmbH. All rights reserved.
//
//  The reproduction, distribution and utilization of this file as
//  well as the communication of its contents to others without express
//  authorization is prohibited. Offenders will be held liable for the
//  payment of damages. All rights reserved in the event of the grant
//  of a patent, utility model or design.
//======================================================================================================================
//  P R O J E C T    I N F O R M A T I O N
//----------------------------------------------------------------------------------------------------------------------
//       Projectname: WAVE3
//  Target system(s):
//       Compiler(s): C++11 compliant
//======================================================================================================================
//  N O T E S
//----------------------------------------------------------------------------------------------------------------------
//  Notes:
//======================================================================================================================
//  I N I T I A L    A U T H O R    I D E N T I T Y
//----------------------------------------------------------------------------------------------------------------------
//        Name: XC-DX/PJ-W3-PER3 Developer
//  Department: XC-DX/PJ-W3-PER3
//======================================================================================================================
/// \swcomponent xloc
/// \file  per_long_dyn.hpp
/// \brief An coding example header file.
//======================================================================================================================
//  C H A N G E    L O G
//----------------------------------------------------------------------------------------------------------------------
//  Version    Author    Description
//  V0.1       MAO Max   Inital version.
//  V1.0       MAO Max   First release according W3 Naming Conventions.
//  V1.1       MAO Max   Rename variable names and function parameters in snake_case.
//======================================================================================================================

//  clang-format off 
//  Generic NAMECPP.1.1: 沿用W1的代码不需要做任何修改，方便之后和W1的修改同步更新。
//                       沿用功能的基础上新开发的功能代码，以文件作为界限：
//                          - 如果是在已有的W1文件上做的修改和更新，维持W1的风格;
//                          - 如果是新加的文件则采用W3的风格。
//
//  Generic NAMECPP.1.2: 使用自我描述的名称，不要使用缩写。名称应该是描述性的和易于理解的。
//                       不要假设每个人都熟悉项目特定的缩写。如果官方 Wikipedia 上记录了缩写，则可以使用。
//                       例如: std::uint32_t numConfigParsingErrors；中'num'是常见且为大家熟悉的缩写。
//                             std::uint32_t cfp; 'cfp'就不是一个可描述的变量名。
//
//  Generic NAMECPP.1.3: 不要注释代码的明显属性。不要简单地描述代码在做什么，提供更抽象的说明代码为什么会这样做。
//
//  Generic NAMECPP.1.4: 使用正确的英文拼写、语法和标点符号。
//
//  Generic NAMECPP.1.5: 源代码中禁止使用关键字'TODO'标记仍需要工作的代码。
//                       deprecated接口需在Doxygen documentation著名，参考格式如下：
//                        ///
//                        /// @deprecated Interface will not be supported any longer from version 1.0 release on, use XXX instead
//                        ///
//
//  File Naming NAMECPP.2.1: 使用带有 componentname_ 的小写字母作为文件的前缀。
//
//  File Naming NAMECPP.2.2: 文件名应与类名相关，全部为小写，componentname_前缀和名称使用下划线字符 （"_"） 连接。
//                           [ File names with lower_snake_case: per_long_dyn.hpp ]
//                           文件名后缀要求：
//                               C source / header files: .c / .h
//                               C++ source / header files: .cpp / .hpp / .inl
//                               template files: Implementation and declaration of templated classes/methods should be
//                                               (preference in this order):in one placein one filein a self contained header.
//
//  Layout NAMECPP.3.1: 每一行的代码字符数不得超过 120 个字符。
//
//  Naming NAMECPP.4.9: 推荐使用#pragma once解决重复包含和避免编译错误，不推荐使用include guards.
//                      使用include guards时将父文件夹名称和类名称包含在guard名称中以实现唯一性，所有字符都大写。
//                      例子如下：
//                          #ifndef <PACKAGE_FOLDER_NAME>_<COMPONENT_FOLDER_NAME>_.._HPP
//                          #define <PACKAGE_FOLDER_NAME>_<COMPONENT_FOLDER_NAME>_.._HPP
//                          ...
//                          #endif // <PACKAGE_FOLDER_NAME>_<COMPONENT_FOLDER_NAME>_.._HPP
//  clang-format on

#pragma once

#ifndef PACKAGE_FOLDER_NAME_COMPONENT_FOLDER_NAME_PER_LONG_DYN_HPP
#define PACKAGE_FOLDER_NAME_COMPONENT_FOLDER_NAME_PER_LONG_DYN_HPP

//  clang-format off 
//  Layout NAMECPP.3.8: 头文件包含按John Lakos include order排列：
//                      1. Prototype/interface header for this implementation (ie, the .h/.hh file that corresponds to this .cpp/.cc file)。 
//                      2. Other headers from the same project, as needed。
//                      3. Headers from other non-standard, non-system libraries (for example, Qt, Eigen, etc)。
//                      4. Headers from other “almost-standard” libraries (for example, Boost)。
//                      5. Standard C++ headers (for example, iostream, functional, etc.)。
//                      6. Standard C headers (for example, cstdint, dirent.h, etc.)。
//                      在每组之后插入一个空行。
//  clang-format on

#include "per/modules/controllers/pme/axSensorPreProc/src/per_axSensorPreProc.hpp"
#include "per/modules/controllers/pme/src/per_pmeInterface.hpp"
#include "per/modules/controllers/pme/vehVelRaw/src/per_vehVelRaw.hpp"
#include "per/modules/controllers/pme/wheels/src/per_wheels.hpp"

#include "dc_interfaces/per/daddy/params/per_pmeParameters.hpp"
#include "vfc/core/vfc_types.hpp"
#include "vfc/linalg/vfc_linalg_matrixmn.hpp"

#include <chrono>
#include <iostream>
#include <sstream>

//  clang-format off 
//  Layout NAMECPP.3.4: 注释使用分隔符"//"，不要使用分隔符"/*...*/"。

//  Layout NAMECPP.3.2: 缩进深度应增加4个空格字符，而不是tab制表符。
//                      例外：不缩进namespace作用域。
//                      例外：类访问级别关键字'public', 'protected', 'private'缩进2个空格符。

//  Naming NAMECPP.4.4: namespace应全部小写且下划线连接 [ Namespaces in lower_snake_case : per_xloc ]

//  Layout NAMECPP.3.3: 始终将左大括号放在一个新行中，并且不要缩进。适用于代码中if...else..., while等左括号。
//                      括号对不要省略，哪怕只有一行代码，甚至else为空也不要省略括号对。
//  clang-format on

namespace per
{
namespace pme
{
// Naming NAMECPP.4.6: const变量名：变量名全大写并使用下划线连接。
constexpr vfc::uint8_t MAX_MOTION_SENSOR_NUMBER = 3u;

// Naming NAMECPP.4.7: 枚举类成员用大写字母编写（UPPER_SNAKE_CASE），多个部分组成的名称使用下划线 ('_') 分隔。
enum class EMemoryProviderError : vfc::uint8_t
{
    /// attempt to add more memory blocks than the capacity allows
    MEMORY_BLOCKS_EXHAUSTED,
    /// an action was performed which requires memory blocks
    NO_MEMORY_BLOCKS_PRESENT,
    /// attempt to create memory although it already was created
    MEMORY_ALREADY_CREATED,
    /// generic error if memory creation failed
    MEMORY_CREATION_FAILED,
    /// an error occurred while getting the page size
    PAGE_SIZE_CHECK_ERROR
};

// clang-format off
// Layout NAMECPP.3.5: 编写预处理器指令以在第1列开始，在'#'和预处理器指令之间使用空格。
//                     每个嵌套级别需在预处理器条件中嵌入4个空格。
// Naming NAMECPP.4.5: 宏名称应全部大写，并带有下划线分隔符。
//                     宏允许两种case下使用：
//                       - 预处理宏;
//                       - 需要字符串化处理; 
// clang-format on

#ifdef PER_WANT_TO_DEFINE
#   define SOME_FLAG
#else
#   define ANOTHER_FLAG
#endif

#define LOG_WARNING(string warning_message) ...

// clang-format off 
// Naming NAMECPP.4.1: 以下声明对于复合类型（枚举、类、结构和联合）的命名是强制性的。 
//                       标识符的名称以表示复合类型的大写字母开头：
//                       'E' 标记一个枚举，enum class EMyEnum : vfc::uint8_t {...};
//                       'C' 标记一个类或结构，class CMyClass; struct CMyStruct;
//                       'T' 标记类模板或结构模板，template <T> TMyTemplateClass;
//                       'U'标记一个联合，union UMyUnion;
//                       如果名称由几个部分组成，则直接连接单个部分。 标识符中的第一个字母和每个后续连接单词的第一个字母大写。
//                     使用 'using' 指令替换 'typedef', 例如，using MyMap = hash_map<InputIterator*, string >;
//                     如果仍使用 'typedef'，请不要在名称前添加或末尾附加任何“_t”或类似名称。
// clang-format on

struct CImuRawData 
{
    vfc::float64_t m_angular_velocity[3u];
    vfc::float64_t m_angular_velocity_covariance[9u];
    vfc::float64_t m_linear_acceleration[3u];
    vfc::float64_t m_linear_acceleration_covariance[9u];
};

class CLongDyn 
{
// clang-format off 
// Layout NAMECPP.3.2: 缩进深度应增加四个空格字符，而不是tab制表符。
//                      例外：不缩进namespace作用域。
//                      例外：类访问级别关键字'public', 'protected', 'private'缩进2个空格符。

// Layout NAMECPP.3.6: 按访问级别以下顺序'public', 'protected', 'private'组织“类”定义。
// clang-format on

  public:
    LongDyn();

    // clang-format off
    // Layout NAMECPP.3.7: 函数布局，返回类型和函数参数应位于同一行上。如果参数列表不适合一行，请将其换行。

    // Naming NAMECPP.4.2: 函数和类成员函数的标识符应遵循 camelCase 命名规则。
    //                     标识符的第一个字母小写，每个后续连接单词的第一个字母大写（“lowerCamelCase”）。

    // Naming NAMECPP.4.3: 具有通用目的的函数和类成员函数命名使用推荐的标识符。
    //                     有些函数有共同的目的，因此，为特殊功能提出了一些标识符或部分标识符，以避免它们的解释导致歧义和混淆：
    //                        get, set - function for accessing one special element
    //                        get... - returns the value of that element
    //                        set... - assigns a value to that element
    //                        is, isNot
    //                        init, terminate - for functions that perform initializing respectively cleaning up
    //                        assign, release - used for objects that use memory managed outside of its scope
    //                        open, close - shall be used on objects with session-like behavior, e.g. files, sessions, connections, devices, etc.
    //                        read, write, get, put - shall be used for reading or writing on objects that appear like a data source or drain, e.g. files
    //                        clear, reset - clears the content of an object or sets it in an initial state; the object is not destructed by this operations
    //                        insert, add, remove - used to add or remove objects of a container without creating or destroying the element
    //                        create, destroy - shall be used on objects that have to allocate/deallocate memory

    // Naming NAMECPP.4.6: 变量和函数参数的标识符采用应满足以下条件：
    //                       标识符由2部分组成，第一部分描述对象的范围， 第二部分是实际名称。
    //                       语法如下: [scope_]"name"
    //                       Scope (prefix):
    //                         m_ - Prefix for private / protected members
    //                         g_ - Prefix for global members
    //                         f_ - Function parameter with function scope
    //                         k_ - Symbolic constant (which have static storage implicitly)
    //                         s_ - Symbol with static storage but not constant (think twice before using!)
    //                         无 - 常规变量命名例子如下std::uint32_t num_messages_in_buffer;
    //                       Name:
    //                         Names for variables are written in lower_snake_case. 
    //
    //                     const变量名：
    //                          constant变量名全大写并使用下划线连接。
    //                          例如, constexpr std::uint8_t CONSTANT_VARIABLE_ALL_UPPERCASE = 42;
    // clang-format on

    /// Calculation of a variance from previous variance and a model value, a predicted value, a model variance
    /// and time constant. If calcVariance is called\, the method shall calculate the variance of the current
    /// signal (val) from the previous signal (stdOld), <a
    /// href="https://rb-alm-13-p-dwa.de.bosch.com:8443/dwa/rm/urn:rational::1-4147106800294823-O-1053-000748a0?doors.view=0000000d">
    /// PJ-DC_SysRS_PER_1053</a>\, <a
    /// href="https://rb-alm-13-p-dwa.de.bosch.com:8443/dwa/rm/urn:rational::1-4147106800294823-O-1056-000748a0?doors.view=0000000d">
    /// PJ-DC_SysRS_PER_1056</a>,
    ///
    vfc::float32_t calcVariance(
        vfc::float32_t f_std_old,
        vfc::float32_t f_val,
        vfc::float32_t f_val_pred,
        vfc::float32_t f_model_variance,
        vfc::float32_t f_tau);
    ///
    ///  \brief Returns the longitudinal velocity
    ///
    vfc::float32_t getLongitudinalVelocity();
    ///
    ///    \brief Sets the longitudinal velocity
    ///
    void setLongitudinalVelocity(vfc::float32_t f_velocity);

  private:
    // clang-format off
    // Naming NAMECPP.4.6: 变量和函数参数的标识符采用应满足以下条件：
    //                       标识符由2部分组成，第一部分描述对象的范围， 第二部分是实际名称。
    //                       语法如下: [scope_]"name"
    //                       Scope (prefix):
    //                         m_ - Prefix for private / protected members
    //                         g_ - Prefix for global members
    //                         f_ - Function parameter with function scope
    //                         k_ - Symbolic constant (which have static storage implicitly)
    //                         s_ - Symbol with static storage but not constant (think twice before using!)
    //                         无 - 常规变量命名例子如下std::uint32_t num_messages_in_buffer;
    //                       Name:
    //                         Names for variables are written in lower_snake_case. 
    //
    //                     const变量名：
    //                          constant变量名全大写并使用下划线连接。
    //                          例如, constexpr std::uint8_t CONSTANT_VARIABLE_ALL_UPPERCASE = 42;
    // clang-format on

    ///
    ///   \brief value of longitudinal velocity
    ///
    vfc::float32_t m_longitudinal_velocity;
    ///
    /// \brief Weight used for dynamic Kalman filter in the longitudinal dynamics.
    ///
    vfc::float32_t m_weight_of_dynamic_model;
    ///
    /// \brief Increment of weight of dynamic filter in model mixing,
    /// if dynamic vehicle movement is detected
    ///
    static constexpr vfc::float32_t k_weight_increment = 1.0F;
};

// Naming NAMECPP.4.8: 模板参数的标识符应满足以下条件：类型参数以后缀“Type”结尾，而值模板参数以后缀“Value”结尾。
//                     标识符中的第一个字母和每个后续连接单词的第一个字母大写。C++11参数包应该使用复数形式的后缀（“Type，Value”）。
// Example: – “Template parameter naming”
// name of type and non-type template parameters
// template <typename ValueType, std::int32_t SizeValue>
// struct TFixedArray
// {
//     ValueType m_data[SizeValue];
// };
//
// name of template parameter pack
// template<typename... ValueTypes>
// class TMyTuple
// {
//     // ...
// };
//
// name of non-type template parameter pack
// template<std::int32_t... InitValues>
// void funcy()
// {
//     vfc::TCArray<std::int32_t, sizeof...(InitValues)> arr {InitValues...};
//     // ...
// }
// clang-format on

//  以下Code Metrics来自RB W3项目对WeRide交付代码的质量要求，PER将集成WeRide的代码，为不降低最终质量，我们的开发过程尽量也遵守这些规则。
//  HIS Metrics 1: 禁止使用goto语句。
//  HIS Metrics 2: 每个函数只能有一个退出点，即每个函数只能有一个return语句。
//                   WeRide may have more than 1 exit point for early return and error handling.
//                   RB allowed WeRide to do multiple returns in the function only in case of parameters check.            
//  HIS Metrics 3: 每个函数代码行数不超过100行
//  HIS Metrics 4: 每个函数调用不同的函数个数不超过12个，但不包括C++标准库提供的函数。
//  HIS Metrics 5: 每个函数内嵌套级别的数量不超过6。
//  HIS Metrics 6: 每个函数圈复杂度低于20.
//                   圈复杂度的计算有一种直观的方法，圈复杂度反映的是“判定条件”的数量，所以圈复杂度实际上就是等于判定节点的数量再加上1。
//                   对应的计算公式为：V (G) = P + 1
//                   其中 P 为判定节点数，常见的判定节点有：
//                     if 语句
//                     while 语句
//                     for 语句
//                     case 语句
//                     catch 语句
//                     and 和 or 布尔操作
//                     ? : 三元运算符
//                   对于多分支的 case 结构或 if - else if - else 结构，统计判定节点的个数时需要特别注意：必须统计全部实际的判定节点数，也即每个 else if 语句，以及每个 case 语句，都应该算为一个判定节点。
//  HIS Metrics 7: 每个函数路径复杂度不超过1000，函数路径NPATH是通过代码的可能路径的总和（忽略循环）。
//  HIS Metrics 8: 函数参数个数不超过7个。
//  HIS Metrics 9: 函数不允许递归调用。

}  // namespace pme
}  // namespace per
#endif  // PACKAGE_FOLDER_NAME_COMPONENT_FOLDER_NAME_PER_LONG_DYN_HPP

