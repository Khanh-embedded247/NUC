#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <std_msgs/msg/string.hpp>
namespace pid
{
  class PidConfig
  {
  public:
    class AbstractParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l, std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }

      virtual void clamp(PidConfig &config, const PidConfig &max, const PidConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const PidConfig &config1, const PidConfig &config2) const = 0;
      virtual void fromServer(std::shared_ptr<rclcpp::Node> node, PidConfig &config) const = 0;
      virtual void toServer(std::shared_ptr<rclcpp::Node> node, const PidConfig &config) const = 0;
      virtual bool fromMessage(const rcl_interfaces::msg::Parameter &param, PidConfig &config) const = 0;
      virtual void toMessage(rcl_interfaces::msg::Parameter &param, const PidConfig &config) const = 0;
      virtual void getValue(const PidConfig &config, boost::any &val) const = 0;

      std::string name;
      std::string type;
      uint32_t level;
      std::string description;
      std::string edit_method;
    };

    typedef std::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef std::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;

    template <class T>
    class ParamDescription : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string name, std::string type, uint32_t level, std::string description,
                       std::string edit_method, T PidConfig::*f)
          : AbstractParamDescription(name, type, level, description, edit_method), field(f)
      {
      }

      T(PidConfig::*field);

      virtual void clamp(PidConfig &config, const PidConfig &max, const PidConfig &min) const
      {
        if (config.*field > max.*field)
          config.*field = max.*field;

        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const PidConfig &config1, const PidConfig &config2) const
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(std::shared_ptr<rclcpp::Node> node, PidConfig &config) const override
      {
        config.*field = node->get_parameter(name).as_double(); // Example assuming double type
      }

      virtual void toServer(std::shared_ptr<rclcpp::Node> node, const PidConfig &config) const override
      {
        node->set_parameter(rclcpp::Parameter(name, config.*field));
      }

      virtual bool fromMessage(const rcl_interfaces::msg::Parameter &param, PidConfig &config) const
      {
        if (param.name == name)
        {
          config.*field = param.value;
          return true;
        }
        return false;
      }

      virtual void toMessage(rcl_interfaces::msg::Parameter &param, const PidConfig &config) const
      {
        param.name = name;
        param.value = config.*field;
      }

      virtual void getValue(const PidConfig &config, boost::any &val) const
      {
        val = config.*field;
      }
    };

    class AbstractGroupDescription
    {
    public:
      AbstractGroupDescription(std::string n, std::string t, int p, int i, bool s)
          : name(n), type(t), parent(p), id(i), state(s)
      {
       
      }

      std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
      bool state;

      virtual void toMessage(rcl_interfaces::msg::Parameter &param, const boost::any &config) const = 0;
      virtual bool fromMessage(const rcl_interfaces::msg::Parameter &param, boost::any &config) const = 0;
      virtual void updateParams(boost::any &cfg, PidConfig &top) const = 0;
      virtual void setInitialState(boost::any &cfg) const = 0;

      void convertParams()
      {
        for (auto &i : abstract_parameters)
        {
          rcl_interfaces::msg::Parameter param;
          i->toMessage(param, nullptr);
          parameters.push_back(param);
        }
      }

      std::string name;
      std::string type;
      int parent;
      int id;

      std::vector<rcl_interfaces::msg::Parameter> parameters;
    };

    typedef std::shared_ptr<AbstractGroupDescription> AbstractGroupDescriptionPtr;
    typedef std::shared_ptr<const AbstractGroupDescription> AbstractGroupDescriptionConstPtr;

    template <class T, class PT>
    class GroupDescription : public AbstractGroupDescription
    {
    public:
      GroupDescription(std::string name, std::string type, int parent, int id, bool state, PT PidConfig::*f)
          : AbstractGroupDescription(name, type, parent, id, state), field(f)
      {
      }

      PT(PidConfig::*field);

      virtual void toMessage(rcl_interfaces::msg::Parameter &param, const boost::any &config) const override
      {
        param.name = name;
        param.type = type;
        param.value = config.*field;
      }

      virtual bool fromMessage(const rcl_interfaces::msg::Parameter &param, boost::any &config) const override
      {
        // Assuming the parameter name matches the group name
        if (param.name == name)
        {
          config.*field = param.value;
          return true;
        }
        return false;
      }

      virtual void updateParams(boost::any &cfg, PidConfig &top) const override
      {
        config_group_t *group = boost::any_cast<config_group_t>(&cfg);
        if (group)
        {
          top.*field = group->state;
        }
      }

      virtual void setInitialState(boost::any &cfg) const override
      {
        config_group_t *group = boost::any_cast<config_group_t>(&cfg);
        if (group)
        {
          group->state = top->*field;
        }
      }
    };

    T(PT::*field);
    std::vector<PidConfig::AbstractGroupDescriptionConstPtr> groups;
  };

  class DEFAULT : public PidConfig
  {
  public:
    DEFAULT()
        : PidConfig("pid_config_node")
    {
      state = true;
      name = "Default";
    }

    void setParams(PidConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if ("Kp_scale" == (*_i)->name)
        {
          Kp_scale = boost::any_cast<double>(val);
        }
        if ("Kp" == (*_i)->name)
        {
          Kp = boost::any_cast<double>(val);
        }
        if ("Ki_scale" == (*_i)->name)
        {
          Ki_scale = boost::any_cast<double>(val);
        }
        if ("Ki" == (*_i)->name)
        {
          Ki = boost::any_cast<double>(val);
        }
        if ("Kd_scale" == (*_i)->name)
        {
          Kd_scale = boost::any_cast<double>(val);
        }
        if ("Kd" == (*_i)->name)
        {
          Kd = boost::any_cast<double>(val);
        }
      }
    }

    double Kp_scale;
    double Kp;
    double Ki_scale;
    double Ki;
    double Kd_scale;
    double Kd;

    bool state;
    std::string name;

  } groups;

  // #line 262
  //"/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
  double Kp_scale;
  // #line 262
  //"/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
  double Kp;
  // #line 262
  //"/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
  double Ki_scale;
  // #line 262
  //"/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
  double Ki;
  // #line 262
  //"/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
  double Kd_scale;
  // #line 262
  //"/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
  double Kd;
  // #line 218
  //"/opt/ros/indigo/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

  bool __fromMessage__(dynamic_reconfigure::Config &msg)
  {
    const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
    const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

    int count = 0;
    for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin();
         i != __param_descriptions__.end(); ++i)
      if ((*i)->fromMessage(msg, *this))
        count++;

    for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin();
         i != __group_descriptions__.end(); i++)
    {
      if ((*i)->id == 0)
      {
        boost::any n = boost::any(this);
        (*i)->updateParams(n, *this);
        (*i)->fromMessage(msg, n);
      }
    }

    if (count != dynamic_reconfigure::ConfigTools::size(msg))
    {
      ROS_ERROR("PidConfig::__fromMessage__ called with an unexpected parameter.");
      ROS_ERROR("Booleans:");
      for (unsigned int i = 0; i < msg.bools.size(); i++)
        ROS_ERROR("  %s", msg.bools[i].name.c_str());
      ROS_ERROR("Integers:");
      for (unsigned int i = 0; i < msg.ints.size(); i++)
        ROS_ERROR("  %s", msg.ints[i].name.c_str());
      ROS_ERROR("Doubles:");
      for (unsigned int i = 0; i < msg.doubles.size(); i++)
        ROS_ERROR("  %s", msg.doubles[i].name.c_str());
      ROS_ERROR("Strings:");
      for (unsigned int i = 0; i < msg.strs.size(); i++)
        ROS_ERROR("  %s", msg.strs[i].name.c_str());
      // @todo Check that there are no duplicates. Make this error more
      // explicit.
      return false;
    }
    return true;
  }

  // This version of __toMessage__ is used during initialization of
  // statics when __getParamDescriptions__ can't be called yet.
  void __toMessage__(dynamic_reconfigure::Config &msg,
                     const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__,
                     const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__) const
  {
    dynamic_reconfigure::ConfigTools::clear(msg);
    for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin();
         i != __param_descriptions__.end(); ++i)
      (*i)->toMessage(msg, *this);

    for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin();
         i != __group_descriptions__.end(); ++i)
    {
      if ((*i)->id == 0)
      {
        (*i)->toMessage(msg, *this);
      }
    }
  }

  void __toMessage__(dynamic_reconfigure::Config &msg) const
  {
    const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
    const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
    __toMessage__(msg, __param_descriptions__, __group_descriptions__);
  }

  void __toServer__(const ros::NodeHandle &nh) const
  {
    const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
    for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin();
         i != __param_descriptions__.end(); ++i)
      (*i)->toServer(nh, *this);
  }

  void __fromServer__(const ros::NodeHandle &nh)
  {
    static bool setup = false;

    const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
    for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin();
         i != __param_descriptions__.end(); ++i)
      (*i)->fromServer(nh, *this);

    const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
    for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin();
         i != __group_descriptions__.end(); i++)
    {
      if (!setup && (*i)->id == 0)
      {
        setup = true;
        boost::any n = boost::any(this);
        (*i)->setInitialState(n);
      }
    }
  }

  void __clamp__()
  {
    const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
    const PidConfig &__max__ = __getMax__();
    const PidConfig &__min__ = __getMin__();
    for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin();
         i != __param_descriptions__.end(); ++i)
      (*i)->clamp(*this, __max__, __min__);
  }

  uint32_t __level__(const PidConfig &config) const
  {
    const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
    uint32_t level = 0;
    for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin();
         i != __param_descriptions__.end(); ++i)
      (*i)->calcLevel(level, config, *this);
    return level;
  }

  static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
  static const PidConfig &__getDefault__();
  static const PidConfig &__getMax__();
  static const PidConfig &__getMin__();
  static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
  static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();

private:
  static const PidConfigStatics *__get_statics__();
};

template <> // Max and min are ignored for strings.
inline void PidConfig::ParamDescription<std::string>::clamp(PidConfig &config, const PidConfig &max,
                                                            const PidConfig &min) const
{
  return;
}
class PidConfigStatics
{
public:
  PidConfigStatics(rclcpp::Node::SharedPtr node) : node_(node)
  {
    // Define parameters
    Kp_scale_param_ = declare_parameter("Kp_scale", 10.0);
    Kp_param_ = declare_parameter("Kp", 0.1);
    Ki_scale_param_ = declare_parameter("Ki_scale", 10.0);
    Ki_param_ = declare_parameter("Ki", 0.1);
    Kd_scale_param_ = declare_parameter("Kd_scale", 10.0);
    Kd_param_ = declare_parameter("Kd", 0.1);

    // Set parameter constraints
    set_parameter_min(Kp_scale_param_, 0.1);
    set_parameter_max(Kp_scale_param_, 100.0);
    set_parameter_min(Kp_param_, 0.0);
    set_parameter_max(Kp_param_, 1.0);
    set_parameter_min(Ki_scale_param_, 0.1);
    set_parameter_max(Ki_scale_param_, 100.0);
    set_parameter_min(Ki_param_, 0.0);
    set_parameter_max(Ki_param_, 1.0);
    set_parameter_min(Kd_scale_param_, 0.1);
    set_parameter_max(Kd_scale_param_, 100.0);
    set_parameter_min(Kd_param_, 0.0);
    set_parameter_max(Kd_param_, 1.0);
  }

private:
  rclcpp::Node::SharedPtr node_;

  // Declare parameters
  rclcpp::Parameter Kp_scale_param_;
  rclcpp::Parameter Kp_param_;
  rclcpp::Parameter Ki_scale_param_;
  rclcpp::Parameter Ki_param_;
  rclcpp::Parameter Kd_scale_param_;
  rclcpp::Parameter Kd_param_;

  // Helper function to declare a parameter
  rclcpp::Parameter declare_parameter(const std::string &name, double value)
  {
    return node_->declare_parameter<double>(name, value);
  }

  // Helper function to set parameter min value
  void set_parameter_min(const rclcpp::Parameter &param, double min_value)
  {
    node_->set_parameter_min_max(param.get_name(), rclcpp::ParameterValue(min_value), rclcpp::ParameterValue(param.as_double()));
  }

  // Helper function to set parameter max value
  void set_parameter_max(const rclcpp::Parameter &param, double max_value)
  {
    node_->set_parameter_min_max(param.get_name(), rclcpp::ParameterValue(param.as_double()), rclcpp::ParameterValue(max_value));
  }
};

// #endif // YOUR_PACKAGE_NAME__PID_CONFIG_STATICS_HPP_