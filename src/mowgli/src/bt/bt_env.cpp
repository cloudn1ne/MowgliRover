/*
 * Mowgli Environment Variable Nodes V1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 *
 * Arguments:
 * 
 *    <GetEnvString var="MY_ENV_VAR" str_out="{envvardata}"/>
 *    <GetEnvInt var="MY_ENV_VAR" int_out="{envvardata}"/>
 *    <GetEnvFloat var="MY_ENV_VAR" float_out="{envvardata}"/>
 * 
 * Description:
 * 
 *    Read environment variable named {var} and saves the output to {str_out}, {int_out}, {float_out}
 * 
 * ------------------------------------------------------------------------------------
 *  
 * For boolean strings like true, false, TruE, FalSe etc you can use
 * 
 * <GetEnvBoolAsInt var="MY_BOOL_ENV_VAR" int_out="{envvardata}"/>
 * 
 * it will map the boolean string to an integer 0 (false) or 1 (true) 
 * 
 * 
 * All GetEnv* function will return FAILURE if the env var given {var} does not exist, and SUCCESS if it does.
 * 
 */

#include "bt_env.h"
#include "behaviortree_cpp_v3/bt_factory.h"

// #define BT_DEBUG 1

/// @brief Get string  from environment variable named {name} and store in {str_out}
/// @return {str_out}
BT::NodeStatus GetEnvString::tick()
{   
    std::string var;
    getInput("var", var);
  
    if ( getenv( var.c_str() ))
    {
        std::string val;
        val = std::string(getenv( var.c_str() ));
#ifdef BT_DEBUG        
    ROS_INFO_STREAM("mowgli_bt: GetEnvString::tick() envvar='" << var << "' = '" << val << "' -> SUCCESS");
#endif        
        setOutput("str_out", val);
        return BT::NodeStatus::SUCCESS;
    }
    else
    {   
#ifdef BT_DEBUG        
    ROS_INFO_STREAM("mowgli_bt: GetEnvString::tick() envvar='" << var << "' -> FAILURE");
#endif                
        setOutput("str_out", std::string(""));
        return BT::NodeStatus::FAILURE;
    }
}


/// @brief Get an integer from environment variable named {name} and store in {str_out}
/// @return {int_out}
BT::NodeStatus GetEnvInt::tick()
{   
    std::string var;
    getInput("var", var);
  
    if ( getenv( var.c_str() ))
    {
        int val;
        val = atoi(getenv( var.c_str() ));
#ifdef BT_DEBUG        
    ROS_INFO_STREAM("mowgli_bt: GetEnvInt::tick() envvar='" << var << "' = " << val << " -> SUCCESS");
#endif        
        setOutput("int_out", val);
        return BT::NodeStatus::SUCCESS;
    }
    else
    {   
#ifdef BT_DEBUG        
    ROS_INFO_STREAM("mowgli_bt: GetEnvInt::tick() envvar='" << var << "' -> FAILURE");
#endif                
        setOutput("int_out", std::string(""));
        return BT::NodeStatus::FAILURE;
    }
}

/// @brief Get a float from environment variable named {name} and store in {str_out}
/// @return {float_out}
BT::NodeStatus GetEnvFloat::tick()
{   
    std::string var;
    getInput("var", var);
  
    if ( getenv( var.c_str() ))
    {
        float val;
        val = atof(getenv( var.c_str() ));
#ifdef BT_DEBUG        
    ROS_INFO_STREAM("mowgli_bt: GetEnvFloat::tick() envvar='" << var << "' = " << val << " -> SUCCESS");
#endif        
        setOutput("float_out", val);
        return BT::NodeStatus::SUCCESS;
    }
    else
    {   
#ifdef BT_DEBUG        
    ROS_INFO_STREAM("mowgli_bt: GetEnvFloat::tick() envvar='" << var << "' -> FAILURE");
#endif                
        setOutput("float_out", std::string(""));
        return BT::NodeStatus::FAILURE;
    }
}

/// @brief Get a bool string from environment variable named {name} and store in {int_out} as either 0 or 1
/// @return {int_out}
BT::NodeStatus GetEnvBoolAsInt::tick()
{   
    std::string var;
    getInput("var", var);
  
    if ( getenv( var.c_str() ))
    {
        int val = 0; // default false

        if (strcasecmp(getenv(var.c_str()), std::string("true").c_str()) == 0)        
        {
            val = 1;
        }

#ifdef BT_DEBUG        
    ROS_INFO_STREAM("mowgli_bt: GetEnvBoolAsInt::tick() envvar='" << var << "' = " << val << " -> SUCCESS");
#endif        
        setOutput("int_out", val);
        return BT::NodeStatus::SUCCESS;
    }
    else
    {   
#ifdef BT_DEBUG        
    ROS_INFO_STREAM("mowgli_bt: GetEnvBoolAsInt::tick() envvar='" << var << "' -> FAILURE");
#endif                
        setOutput("int_out", std::string(""));
        return BT::NodeStatus::FAILURE;
    }
}

