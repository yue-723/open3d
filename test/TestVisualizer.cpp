// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <iostream>
#include <memory>
#include <thread>
#include <string>
#include <Open3D/Open3D.h>
#include <fstream>
#include <windows.h>
#include <vector>
#include <sstream>
#include <map>

using namespace open3d;
using namespace std;


class csvfile;

inline static csvfile& endrow(csvfile& file);
inline static csvfile& flush(csvfile& file);

class csvfile
{
    std::ofstream fs_;
    const std::string separator_;
public:
    csvfile(const std::string filename, const std::string separator = ",")
        : fs_()
        , separator_(separator)
    {
        fs_.exceptions(std::ios::failbit | std::ios::badbit);
        fs_.open(filename);
    }

    ~csvfile()
    {
        flush();
        fs_.close();
    }

    void flush()
    {
        fs_.flush();
    }

    void endrow()
    {
        fs_ << std::endl;
    }

    csvfile& operator << (csvfile& (*val)(csvfile&))
    {
        return val(*this);
    }

    csvfile& operator << (const char* val)
    {
        fs_ << '"' << val << '"' << separator_;
        return *this;
    }

    csvfile& operator << (const std::string& val)
    {
        fs_ << '"' << val << '"' << separator_;
        return *this;
    }

    template<typename T>
    csvfile& operator << (const T& val)
    {
        fs_ << val << separator_;
        return *this;
    }
};
inline static csvfile& endrow(csvfile& file)
{
    file.endrow();
    return file;
}

inline static csvfile& flush(csvfile& file)
{
    file.flush();
    return file;
}
//sub func prototype
void display_Inlier_Outlier(shared_ptr<open3d::geometry::PointCloud>, vector<size_t>);

extern "C" _declspec(dllexport) void Remove_Outlier(const char*, int, size_t, double, double);
extern "C" _declspec(dllexport) void ScrShot(const char*, int);

auto pcd = std::make_shared<geometry::PointCloud>();



string SaveFilePath(char* filter = "All Files (*.*)\0*.*\0", HWND owner = NULL) {
    OPENFILENAME saveFileDialog;
    char szSaveFileName[MAX_PATH] = { 0 };
    ZeroMemory(&saveFileDialog, sizeof(saveFileDialog));
    saveFileDialog.lStructSize = sizeof(saveFileDialog);
    saveFileDialog.hwndOwner = owner;
    saveFileDialog.lpstrFilter = "All Files (*.*)\0*.*\0";
    saveFileDialog.lpstrFile = szSaveFileName;
    saveFileDialog.nMaxFile = MAX_PATH;
    saveFileDialog.Flags = OFN_EXPLORER | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT;

    if (GetSaveFileName(&saveFileDialog)) {
        return saveFileDialog.lpstrFile;
    }
    else
        return "Error occured";
}

int main() {
}
void display_Inlier_Outlier(shared_ptr<open3d::geometry::PointCloud> pcd1, vector<size_t> ind) {
    auto inlierCloud = pcd1->SelectByIndex(ind);
    auto OutlierCloud = pcd1->SelectByIndex(ind, true);
    inlierCloud->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
    OutlierCloud->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
    visualization::DrawGeometries({ inlierCloud,OutlierCloud }, "PointCloud", 1024, 720);
}
void display_Inlier_Outlier2(shared_ptr<open3d::geometry::PointCloud> pcd1, shared_ptr<open3d::geometry::PointCloud> pcd2)
{    
    auto OriColor = pcd2->colors_;
    pcd2->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
    pcd1->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
    visualization::DrawGeometries({ pcd1,pcd2 }, "PointCloud", 1024, 720);
    pcd2->colors_ = OriColor;
}

extern "C" _declspec(dllexport) void Remove_Outlier(const char* input_fn,int removeMethod, size_t nb_neighbors,double std_ratio,double thres)
{
    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

    int idx = string(input_fn).find_last_of(".") + 1;
    string fileType = string(input_fn).substr(idx);

    if (fileType == "csv")
    {
        vector<Eigen::Vector3d> points;
        vector<Eigen::Vector3d> colors;

        int point_id = 0;   //Point(x,y,z) 's col
        int color_id = 0;   //Color(r,g,b) 's col
        string line;
        bool firstLine = true;
        fstream file;
        file.open(input_fn);

        if (file)
            utility::LogInfo("Successfully read {}\n", input_fn);
        
        else
            utility::LogWarning("Failed to read {}\n\n", input_fn);
        
        while (getline(file, line, '\n'))
        {
            istringstream eachLine(line);
            string data;

            vector<string> head;
            vector<double> content;
            vector<double> tempXYZ;
            vector<double> tempRGB;
            bool getIndex = false;

            if (firstLine)
            {
                while (getline(eachLine, data, ','))
                {
                    data.erase(remove_if(data.begin(), data.end(), isspace), data.end());
                    data.erase(remove(data.begin(), data.end(), '"'), data.end());
                    head.push_back(data);
                    for (int i = 0; i < head.size(); i++)
                    {
                        if (head[i] == "X" || head[i] == "x")
                            point_id = i;
                        else if (head[i] == "R" || head[i] == "r")
                            color_id = i;
                    }
                }
                firstLine = false;
            }
            else
            {
                while (getline(eachLine, data, ','))
                {
                    data.erase(remove_if(data.begin(), data.end(), isspace), data.end());
                    content.push_back(stod(data));
                }
                points.push_back(Eigen::Vector3d(content[point_id], content[point_id + 1], content[point_id + 2]));
                points.push_back(Eigen::Vector3d(content[color_id], content[color_id + 1], content[color_id + 2]));
                content.clear();
            }
        }
        file.close();
        pcd->points_ = points;
        pcd->colors_ = colors;
    }
    else if (fileType == "txt" || fileType == "xyzrgb")
    {
        if (io::ReadPointCloud(input_fn, *pcd, io::ReadPointCloudOption::ReadPointCloudOption("xyzrgb"))) {
            utility::LogInfo("Successfully read {}\n", input_fn);
        }
        else {
            utility::LogWarning("Failed to read {}\n\n", input_fn);
        }
    }
    else if (fileType == "ply" || fileType == "pcd")
    {
        if (io::ReadPointCloud(input_fn, *pcd)) {
            utility::LogInfo("Successfully read {}\n", input_fn);
        }
        else {
            utility::LogWarning("Failed to read {}\n\n", input_fn);
        }
    }

    pcd->NormalizeNormals();
    // ind = inlier point index after removing outlier
    if (removeMethod == 1)
    {
        cout << "RemoveStatistical\n";
        visualization::DrawGeometries({ pcd }, "PointCloud", 1024, 720);
        auto ind = get<1>(pcd->RemoveStatisticalOutliers(nb_neighbors, std_ratio, true));
        display_Inlier_Outlier(pcd, ind);
        pcd = pcd->SelectByIndex(ind);
        visualization::DrawGeometries({ pcd }, "PointCloud", 1024, 720);
    }
    else if (removeMethod == 2)
    {
        cout << "Multi_RemoveStatistical\n";
        visualization::DrawGeometries({ pcd }, "PointCloud", 1024, 720);
        auto ind = get<1>(pcd->RemoveStatisticalOutliers(nb_neighbors, std_ratio, true));
        double totalRemovedPts = pcd->SelectByIndex(ind, true)->points_.size();
        auto inlierCloud = pcd->SelectByIndex(ind);
        int removingTimes = 1;
        cout << removingTimes << ". removed points: " << totalRemovedPts
            << " inlier points: " << ind.size() << " R/O: " << (totalRemovedPts / pcd->points_.size()) << endl;
        while ((totalRemovedPts / pcd->points_.size()) <= thres)
        {
            auto removedIND = get<1>(inlierCloud->RemoveStatisticalOutliers(nb_neighbors, std_ratio, true));
            auto removedPCD = inlierCloud->SelectByIndex(removedIND);
            totalRemovedPts = pcd->points_.size() - removedPCD->points_.size();
            inlierCloud = removedPCD;
            removingTimes++;
            cout << removingTimes << ". removed points: " << totalRemovedPts
                << " inlier points: " << inlierCloud->points_.size() << " R/O: " << (totalRemovedPts / pcd->points_.size()) << endl;
        }
        //display_Inlier_Outlier2(pcd, inlierCloud);
        pcd = inlierCloud;
        visualization::DrawGeometries({ pcd }, "PointCloud", 1024, 720);
    }
    else if (removeMethod == 3)
    {
        cout << "RemoveRadius\n";
        visualization::DrawGeometries({ pcd }, "PointCloud", 1024, 720);
        auto ind = get<1>(pcd->RemoveRadiusOutliers(nb_neighbors, std_ratio, true));
        display_Inlier_Outlier(pcd, ind);
        pcd = pcd->SelectByIndex(ind);
        visualization::DrawGeometries({ pcd }, "PointCloud", 1024, 720);
    }
    else if (removeMethod == 4)
    {
        cout << "Multi_RemoveRadius\n";
        visualization::DrawGeometries({ pcd }, "PointCloud", 1024, 720);
        auto ind = get<1>(pcd->RemoveRadiusOutliers(nb_neighbors, std_ratio, true));
        double totalRemovedPts = pcd->SelectByIndex(ind, true)->points_.size();
        auto inlierCloud = pcd->SelectByIndex(ind);
        int removingTimes = 1;
        cout << removingTimes << ". removed points: " << totalRemovedPts
            << " inlier points: " << ind.size() << " R/O: " << (totalRemovedPts / pcd->points_.size()) << endl;
        while ((totalRemovedPts / pcd->points_.size()) <= thres)
        {
            auto removedIND = get<1>(inlierCloud->RemoveRadiusOutliers(nb_neighbors, std_ratio, true));
            auto removedPCD = inlierCloud->SelectByIndex(removedIND);
            totalRemovedPts = pcd->points_.size() - removedPCD->points_.size();
            inlierCloud = removedPCD;
            removingTimes++;
            cout << removingTimes << ". removed points: " << totalRemovedPts
                << " inlier points: " << inlierCloud->points_.size() << " R/O: " << (totalRemovedPts / pcd->points_.size()) << endl;
        }
        //display_Inlier_Outlier2(pcd, inlierCloud);
        pcd = inlierCloud;
        visualization::DrawGeometries({ pcd }, "PointCloud", 1024, 720);
    }
    string filePath = SaveFilePath().c_str();
    if (filePath != "Error occured")
    {
        cout << filePath << endl;
        string output_ply_fp = filePath + "_denoised.ply";
        string output_csv_fp = filePath + "_denoised.csv";

        io::WritePointCloud(output_ply_fp, *pcd);
        auto points = pcd->points_;
        auto colors = pcd->colors_;
        vector<Eigen::Vector3d> pcdData(points);
        pcdData.insert(pcdData.end(), colors.begin(), colors.end());

        utility::LogInfo("Saving as csv\n");
        csvfile csv(output_csv_fp);

        // Header
        csv << "X" << "Y" << "Z" << "R" << "G" << "B" << endrow;
        // Data
        for (int i = 0; i < pcdData.size(); i++)
        {
            for (int j = 0; j < 6; j++)
                csv << pcdData[i][j];
            csv << endrow;
        }
        utility::LogInfo("Save csv successfully\n");
    }
    else
        utility::LogInfo("Save Error occured,check the file name\n");
}

extern "C" _declspec(dllexport) void ScrShot(const char* input_fn, int vp)
{
    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    pcd->NormalizeNormals();
    if (io::ReadPointCloud(input_fn, *pcd)) {
        utility::LogInfo("Successfully read {}\n", input_fn);
    }
    else {
        utility::LogWarning("Failed to read {}\n\n", input_fn);
    }
    visualization::VisualizerWithKeyCallback vis;

    vis.RegisterKeyActionCallback(GLFW_KEY_P,
        [&](visualization::Visualizer*, int action, int mods) {
            if (action == 0)
            {
                string filePath = SaveFilePath().c_str();
                if (filePath != "Error occured")
                {
                    filePath += "_ScrShot.png";
                    vis.CaptureScreenImage(filePath, true);
                }
                else
                    utility::LogInfo("Save Error occured,check the file name\n");
            }
            return true;
        });

    vis.CreateVisualizerWindow("Open3D ScrShot");
    vis.AddGeometry(pcd);
    auto& view_ctl = vis.GetViewControl();
    switch (vp)
    {
    case 0:                                             //"Top"
        view_ctl.SetUp(Eigen::Vector3d(0, 1, 0));
        break;
    case 1:                                             //"Bottom"
        view_ctl.SetFront(Eigen::Vector3d(0, 0, -5));
        break;
    case 2:                                             //"Front"
        view_ctl.SetFront(Eigen::Vector3d(1, -5, 0));
        break;
    case 3:                                             //"Back"
        view_ctl.SetFront(Eigen::Vector3d(-1, 5, 0));
        break;
    case 4:                                             //"Left"
        view_ctl.SetFront(Eigen::Vector3d(-1, 0, 0));
        break;
    case 5:                                             //"Right"
        view_ctl.SetFront(Eigen::Vector3d(1, 0, 0));
        break;
    case 6:                                             //"ISO_VIEW_1"
        view_ctl.SetUp(Eigen::Vector3d(1, 1, 0));
        view_ctl.SetFront(Eigen::Vector3d(1, 1, 1));
        break;
    case 7:                                             //"ISO_VIEW_2"
        view_ctl.SetFront(Eigen::Vector3d(0, 1, 1));
        break;
    default:
        break;
    }
   
    vis.UpdateGeometry(pcd);
    vis.PollEvents();
    vis.UpdateRender();
    vis.Run();
    vis.DestroyVisualizerWindow();
}