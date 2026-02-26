#ifndef UUV_INTERFACE_XML_PARAM_READER_H
#define UUV_INTERFACE_XML_PARAM_READER_H

#include <string>
#include <vector>
#include <sstream>
#include <tinyxml.h>
#include <Eigen/Dense>
#include <ros/ros.h>

namespace uuv_interface {

class XmlParamReader {
private:
    TiXmlDocument doc_;
    TiXmlElement* root_ = nullptr;

public:
    // 从字符串构造 (用于解析传给插件的 xml_snippet)
    XmlParamReader(const std::string& xml_str) {
        doc_.Parse(xml_str.c_str());
        root_ = doc_.RootElement();
    }

    // 从节点构造 (用于嵌套标签的子读取器，如 <thruster> 和 <fin>)
    XmlParamReader(TiXmlElement* elem) : root_(elem) {}

    // 检查标签是否有效
    bool isValid() const { return root_ != nullptr; }

    // 获取标签属性 (如 name="sonar_left" 或 enable_dynamic_reconfigure="true")
    std::string getAttribute(const std::string& attr_name, const std::string& default_val = "") {
        if (root_ && root_->Attribute(attr_name.c_str())) {
            return std::string(root_->Attribute(attr_name.c_str()));
        }
        return default_val;
    }

    // 获取子节点读取器 (用于嵌套解析)
    XmlParamReader getChild(const std::string& tag_name) {
        if (!root_) return XmlParamReader(nullptr);
        return XmlParamReader(root_->FirstChildElement(tag_name.c_str()));
    }

    // ====================================================
    // 重载的 param() 函数：风格完全模仿 ROS 的 gnh.param()
    // ====================================================

    void param(const std::string& tag_name, double& val, double default_val) {
        val = default_val;
        if (!root_) return;
        TiXmlElement* e = root_->FirstChildElement(tag_name.c_str());
        if (e && e->GetText()) val = std::stod(e->GetText());
    }

    void param(const std::string& tag_name, int& val, int default_val) {
        val = default_val;
        if (!root_) return;
        TiXmlElement* e = root_->FirstChildElement(tag_name.c_str());
        if (e && e->GetText()) val = std::stoi(e->GetText());
    }

    void param(const std::string& tag_name, bool& val, bool default_val) {
        val = default_val;
        if (!root_) return;
        TiXmlElement* e = root_->FirstChildElement(tag_name.c_str());
        if (e && e->GetText()) {
            std::string s(e->GetText());
            
            // 1. 去除首尾的空白字符（空格、换行、制表符等）
            s.erase(0, s.find_first_not_of(" \n\r\t"));
            s.erase(s.find_last_not_of(" \n\r\t") + 1);
            
            // 2. 将字符串转换为小写（兼容 "True", "TRUE" 等情况）
            std::transform(s.begin(), s.end(), s.begin(), ::tolower);
            
            // 3. 重新进行判断
            val = (s == "true" || s == "1");
        }
    }

    void param(const std::string& tag_name, std::string& val, const std::string& default_val) {
        val = default_val;
        if (!root_) return;
        TiXmlElement* e = root_->FirstChildElement(tag_name.c_str());
        if (e && e->GetText()) val = std::string(e->GetText());
    }

    void param(const std::string& tag_name, std::vector<double>& vec, const std::vector<double>& default_val) {
        vec = default_val;
        if (!root_) return;
        TiXmlElement* e = root_->FirstChildElement(tag_name.c_str());
        if (e && e->GetText()) {
            std::stringstream ss(e->GetText());
            double v; vec.clear();
            while (ss >> v) vec.push_back(v);
        }
    }

    void paramMatrix(const std::string& tag_name, Eigen::MatrixXd& mat, int rows, int cols) {
        if (!root_) return;
        TiXmlElement* e = root_->FirstChildElement(tag_name.c_str());
        if (e && e->GetText()) {
            std::stringstream ss(e->GetText());
            std::vector<double> v; double val;
            while (ss >> val) v.push_back(val);
            if (v.size() == rows * cols) {
                for(int i=0; i<rows; ++i) {
                    for(int j=0; j<cols; ++j) {
                        mat(i,j) = v[i*cols+j];
                    }
                }
            } else {
                ROS_WARN_STREAM("[XmlParamReader] Matrix dimension mismatch for tag: " << tag_name);
            }
        }
    }
};

} // namespace uuv_interface
#endif