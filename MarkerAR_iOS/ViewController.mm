#import "ViewController.h"
#import <opencv2/opencv.hpp>
#import <opencv2/imgproc/types_c.h>
#import <opencv2/videoio/cap_ios.h>
#include <iostream>
#include "MarkerRecognizer.h"

using namespace cv;
using namespace std;

Point3f corners_3d[] =
{
    Point3f(-0.35f, -0.35f, 0),
    Point3f(-0.35f,  0.35f, 0),
    Point3f( 0.35f,  0.35f, 0),
    Point3f( 0.35f, -0.35f, 0)
};

float f_x = 640.0f;
float f_y = 640.0f;
float c_x = 320.0f;
float c_y = 240.0f;

float camera_matrix[] =
{
    f_x, 0.0f, c_x,
    0.0f, f_y, c_y,
    0.0f, 0.0f, 1.0f
};

float dist_coeff[] = {0.0f, 0.0f, 0.0f, 0.0f};
vector<Point3f> m_corners_3d = vector<Point3f>(corners_3d, corners_3d + 4);

@interface ViewController ()
{
    
    CvVideoCamera* videoCamera;
    MarkerRecognizer m_recognizer;
    
}
@end

@implementation ViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view, typically from a nib.
    UIImage *image = [UIImage imageNamed:@"timg.jpeg"];
    UIImageView *imageView = [[UIImageView alloc] initWithImage:image];
    imageView.frame=CGRectMake(10, 40,360, 550);
    //imageView.backgroundColor = [UIColor yellowColor];
    [self.view addSubview:imageView];
    /*
    UIButton * button = [UIButton buttonWithType:UIButtonTypeCustom];// 自定义button,现在oc追求扁平化，有些效果
    [button setTitle:@"Start" forState:UIControlStateNormal];

    button.frame = CGRectMake(130, 610, 100, 45) ;

    button.backgroundColor = [UIColor blueColor];
    //    button.highlighted = YES;
    
    //    button.enabled = YES;
    [button addTarget:self action:@selector(changeImage) forControlEvents:UIControlEventTouchUpInside];
    [self.view addSubview:button];
    
    button.contentHorizontalAlignment = UIControlContentHorizontalAlignmentRight ;// 右对齐
    
    UIEdgeInsets Imageinset = {0,10,0,0};// 上左下右
    
    [button setImageEdgeInsets:Imageinset];
    
    UIEdgeInsets titleInset = {0,20,20,0};
    
    [button setTitleEdgeInsets:titleInset];
    
    
    */
    self->videoCamera = [[CvVideoCamera alloc] initWithParentView:imageView];
    self->videoCamera.delegate = self;
    self->videoCamera.defaultAVCaptureDevicePosition = AVCaptureDevicePositionBack;
    self->videoCamera.defaultAVCaptureSessionPreset = AVCaptureSessionPreset640x480;
    self->videoCamera.defaultFPS = 30;
    self->videoCamera.grayscaleMode = NO;
    
}
- (IBAction)change:(id)sender {
     [self->videoCamera start];
}

- (void)changeImage{
    
   
    
}

- (void)processImage:(cv::Mat &)image
{
    
    Mat imgTemp_gray;
    cvtColor(image, imgTemp_gray, CV_BGRA2GRAY);
    m_recognizer.update(imgTemp_gray, 100);
    vector<Marker>& markers = m_recognizer.getMarkers();
    
    m_recognizer.drawToImage(image, Scalar(255,0,0,255), 2);
    Mat intrinsics =Mat(3, 3, CV_32FC1, camera_matrix);
    Mat distortion =Mat(1, 4, CV_32FC1, dist_coeff);
    vector<Point3f> objectPoints;
    objectPoints.push_back(Point3f(-1, 1, 0));
    objectPoints.push_back(Point3f(1, 1, 0));
    objectPoints.push_back(Point3f(1, -1, 0));
    objectPoints.push_back(Point3f(-1, -1, 0));
    Mat objectPointsMat(objectPoints);
    
    Mat rvec;
    Mat tvec;
    if(m_recognizer.getMarkers().size()!=0)
    {
        for(int i=0;i<m_recognizer.getMarkers().size();i++)
        {
            solvePnP(objectPointsMat, m_recognizer.getMarkers()[i].m_corners, intrinsics, distortion, rvec, tvec);
            
            cout << "rvec: " << rvec << endl;
            cout << "tvec: " << tvec << endl;
            
            vector<Point3f> line3dx = {{0, 0, 0}, {2, 0, 0}};
            vector<Point3f> line3dy = {{0, 0, 0}, {0, 2, 0}};
            vector<Point3f> line3dz = {{0, 0, 0}, {0, 0, -2}};
            
            vector<Point2f> line2dx;
            vector<Point2f> line2dy;
            vector<Point2f> line2dz;
            projectPoints(line3dx, rvec, tvec, intrinsics, distortion, line2dx);
            projectPoints(line3dy, rvec, tvec, intrinsics, distortion, line2dy);
            projectPoints(line3dz, rvec, tvec, intrinsics, distortion, line2dz);
            
            
            line(image, line2dx[0], line2dx[1], Scalar(255,0,0),10);
            line(image, line2dy[0], line2dy[1], Scalar(0,255,0),10);
            line(image, line2dz[0], line2dz[1], Scalar(0,0,255),10);
        }
    }
    
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}


@end
