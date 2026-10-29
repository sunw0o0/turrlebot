# parking_node: GUI 파라미터 탭에서 바꾼 값을 바로 적용 (다시 켜지 않아도 됨)
#   parking 폴더 (colcon_ws/src/parking) 에서 실행. 다른 패치를 더 넣은 뒤에는 이 스크립트를 다시 실행하면
#   새 파라미터까지 다시 만들어 준다 (이전에 만든 부분은 지우고 새로)
import re, sys
n = 'src/parking_node.cpp'
ns = open(n).read()
BEGIN = '  // ---- live_params: 시작 (자동 생성, 직접 고치지 말 것) ----\n'
END = '  // ---- live_params: 끝 ----\n'
REG_BEGIN = '    // ---- live_params 등록 ----\n'
REG_END = '    // ---- live_params 등록 끝 ----\n'
MEM = "  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr live_params_handle_;   // live_params\n"
# 예전에 만든 부분 지우기 (다시 실행할 때)
if BEGIN in ns:
    ns = ns[:ns.index(BEGIN)] + ns[ns.index(END) + len(END):]
if REG_BEGIN in ns:
    ns = ns[:ns.index(REG_BEGIN)] + ns[ns.index(REG_END) + len(REG_END):]
ns = re.sub(r'^.*live_params_handle_;.*\n', '', ns, flags=re.M)

A = '    declare_all_parameters();\n'
B = '  bool i_am_driving() const\n'
C = '  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;\n'
for key in (A, B, C):
    if ns.count(key) != 1:
        print('파일이 예상과 달라요. 바꾸지 않았어요:', key.strip()[:50]); sys.exit()

pairs = re.findall(r'^\s+([a-z_0-9]+_) = declare_parameter\("([a-z_0-9]+)",', ns, re.M)
skip = {'image_topic'}
lines = []
for member, name in pairs:
    if name in skip:
        continue
    extra = ''
    if name == 'entry_arc_speed' and 'entry_arc_speed_base_' in ns:
        extra = ' entry_arc_speed_base_ = entry_arc_speed_;'
    if name == 'exit_arc_speed' and 'exit_arc_speed_base_' in ns:
        extra = ' exit_arc_speed_base_ = exit_arc_speed_;'
    lines.append(f'    if (name == "{name}") {{ set_from(p, {member});{extra} return true; }}\n')

fn = BEGIN + '''  // GUI (set_parameters) 로 바꾼 값을 멤버 변수에 바로 넣는다. 모르는 이름이면 false
  template<class T>
  static void set_from(const rclcpp::Parameter & p, T & dst)
  {
    if constexpr (std::is_same_v<T, bool>)
    {
      dst = p.as_bool();
    }
    else if constexpr (std::is_integral_v<T>)
    {
      dst = static_cast<T>(p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE ? p.as_double() : p.as_int());
    }
    else if constexpr (std::is_floating_point_v<T>)
    {
      dst = static_cast<T>(p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ? p.as_int() : p.as_double());
    }
    else
    {
      dst = p.as_string();
    }
  }

  bool apply_param(const rclcpp::Parameter & p)
  {
    const std::string & name = p.get_name();
''' + ''.join(lines) + '''    return false;
  }

''' + END
ns = ns.replace(B, fn + B)

ns = ns.replace(A, A + REG_BEGIN + '''    // GUI 파라미터 탭에서 값을 바꾸면 바로 적용 (다음 주차 동작부터. 도는 중이면 그 다음 판단부터)
    live_params_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params)
      {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : params)
        {
          try
          {
            if (apply_param(p))
            {
              RCLCPP_INFO(get_logger(), "파라미터 바로 적용: %s = %s", p.get_name().c_str(), p.value_to_string().c_str());
            }
          }
          catch (const std::exception & e)
          {
            result.successful = false;
            result.reason = p.get_name() + ": " + e.what();
          }
        }
        return result;
      });
''' + REG_END)
ns = ns.replace(C, C + MEM)
if '#include <type_traits>' not in ns:
    ns = ns.replace('#include <vector>\n', '#include <type_traits>\n#include <vector>\n', 1)
open(n, 'w').write(ns)
print(f'GUI 값 바로 적용 추가 OK ({len(lines)}개 파라미터)')
