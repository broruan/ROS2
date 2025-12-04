#include<iostream>
#include<opencv2/opencv.hpp>
#include<vector>
#include<cmath>
#include<map>
#include<utility>

float Distance(const cv::Point A,const cv::Point B){
    return sqrt(abs(A.x-B.x)*abs((A.x-B.x))+abs(A.y+B.y)*abs(A.y+B.y));
}


int main(){
    std::vector<cv::Mat> channels;
    cv::Mat img = cv::imread("../Robot.png");
    if(img.empty()){
        std::cout<<"Loading img error"<<std::endl;
        return -1;
    }
    else{
        cv::split(img,channels);
        // cv::Mat blue = channels[0];
        // cv::Mat green = channels[1];
        // cv::Mat red = channels[2];


        cv::Mat blue = channels[0],dst,can,dil;
        blue -= channels[2];
        // Gauss滤波器
        cv::GaussianBlur(blue,dst,cv::Size(5,5),0,0);

        // Canny
        cv::Canny(dst,can,220,240);

        // 获取自定义核
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

        // 膨胀操作
        cv::dilate(can,dil,element);

        // 寻找轮廓
        std::vector<std::vector<cv::Point>> contours;
        std::vector<std::vector<cv::Point>> curve;
        std::vector<cv::Vec4i> h;
        std::vector<cv::Point> approx;
        std::vector<cv::Point> top,bot;
        // std::map<float,std::vector<vector<cv::Point>>> pointDistance;
        // std::pair<float,std::vector<std::vector<cv::Point>>> i_pair;
        cv::findContours(dil,contours,h,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);

        // 绘制轮廓

        // 多边形近似
        // for(int i=0;i<contours.size();i++){
        //     if(cv::contourArea(contours[i])>800){
        //         double ep = 0.05*cv::arcLength(contours[i],true);
        //         cv::approxPolyDP(contours[i],approx,ep,true);
        //         curve.push_back(approx);
        //         approx.clear();
        //     }
            // double area = cv::contourArea(contours[i]);
            // std::cout<<"第"<<i<<"个轮廓:"<<area<<std::endl;
            // else{
            //     contours.clear();
            // }
        // }
        for (size_t i = 0; i < contours.size(); i++) {
            double lens = cv::arcLength(contours[i],true);
            std::cout<<"第"<<i<<"个轮廓周长:"<<lens<<std::endl;
            if(cv::contourArea(contours[i])>200 && lens>190){
                cv::Rect rect = cv::boundingRect(contours[i]);// Rect结构体(x,y,width,height)
                top.push_back(cv::Point ((2*rect.x+rect.width)/2,rect.y));
                bot.push_back(cv::Point ((2*rect.x+rect.width)/2,rect.y+rect.height));
                cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 3);
                // cv::circle(img,top[0],5,cv::Scalar(0,255,0),-1);
                // cv::circle(img,bot[0],5,cv::Scalar(0,255,0),-1);
                // cv::circle(img,top[1],5,cv::Scalar(0,255,0),-1);
                // cv::circle(img,bot[1],5,cv::Scalar(0,255,0),-1);

            }
        }

        float Max=0;
        float Min=100000;
        int a,b;
        for(int i = 0;i < top.size();i++){
            for(int f = 0;f < top.size();f++){
                if(i!=f){
                   float dis = Distance(top[i],top[f]);
                   if(Max<dis && abs(top[i].y-top[f].y)<Min){
                    Max = dis;
                    Min = abs(top[i].y-top[f].y);
                    a=i;
                    b=f;
                   }
                }
            }

            cv::circle(img,top[i],5,cv::Scalar(0,255,0),-1);
            cv::circle(img,bot[i],5,cv::Scalar(0,255,0),-1);
        }

        cv::line(img,top[a],bot[b],cv::Scalar(255,0,0),2,cv::LINE_AA);
        cv::line(img,bot[a],top[b],cv::Scalar(255,0,0),2,cv::LINE_AA);
        cv::Point mid = ((top[a]+bot[a])/2 + (top[b]+bot[b])/2)/2;
        cv::circle(img,mid,5,cv::Scalar(203,192,255),-1);
        top.clear();
        bot.clear();
        contours.clear();

        cv::imshow("原图",img);
        cv::imshow("blue",blue);
        cv::imshow("blur",dst);
        cv::imshow("Canny",can);
        cv::imshow("Dilate",dil);
        // cv::imshow("green",channels[1]);
        // cv::imshow("red",channels[2]);
        cv::imwrite("../result.jpeg",img);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
    return 0;
}



// 未实现功能：1.判断两个点（top内和bot内比较）相距最远^^
//           2.判断完之后，取同一组中top和bot中点，再取相距最远的两组的中点连线的中点。^^


//不具有健壮性：取交叉角点时应该取矩形两点的中点。