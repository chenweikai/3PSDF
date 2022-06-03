#include "computeVisualHull.h"

// camera paramters
float l = -55;      // left
float r = 55;       // right
float b = -55;      // bottom
float t = 55;       // top
float near = 0.1;        // near
float far = 10000;  // far

// for computing lookat matrix
Vector3d eye = Vector3d(0, 0, 2000);
Vector3d center = Vector3d(0, 0, 0);
Vector3d up = Vector3d(0, 1, 0);

// image size
int img_height = 500;
int img_width = 500;

string imgName = "/home/weikai/data/mixamo_mesh_and_render/uriel_a_plotexia/uriel_a_plotexia_0.png";
// string meshName = "/home/weikai/data/mixamo_mesh_and_render/uriel_a_plotexia/uriel_a_plotexia.obj";
vector<Vector3d> viewPoints({Vector3d(0, 0, 2000), Vector3d(0, 0, -2000), Vector3d(2000, 0, 0), Vector3d(-2000, 0, 0)});
string meshReconName = "../output/recon.obj";

// calculate the l2 norm between query pixel and background color (bgcolor)
double calDiff(const CImg<unsigned char>& img, const Vector3i& bgColor, const Vector2i& pos)
{
    int x = pos[0], y = pos[1];
    double diff = (img(x,y,0,0) - bgColor[0]) * (img(x,y,0,0) - bgColor[0])
                    + (img(x,y,0,1) - bgColor[1]) * (img(x,y,0,1) - bgColor[1])
                    + (img(x,y,0,2) - bgColor[2]) * (img(x,y,0,2) - bgColor[2]);
    diff = sqrt(diff);
    return diff;
}

void dfsMaskBackgroundPixels(const Vector2i& pos, const CImg<unsigned char>& input,
    CImg<unsigned char>& output, CImg<unsigned char>& visited, 
    const Vector3i& bgColor, const double& threshold, int& counter, 
    const int& height, const int& width)
{
    if (pos[0] < 0 || pos[0] >= width || pos[1] < 0 || pos[1] >= height)
        return;
    if ((int)visited(pos[0], pos[1], 0, 0) == 1)
        return;
    // mask current pixel as visited    
    visited(pos[0], pos[1], 0, 0) = 1;

    // if current pixels is similar with background color within a threshold
    // mask it as transparent
    // cout << "current pos: " << pos[0] << " " << pos[1] << endl;
    if (calDiff(input, bgColor, pos) <= threshold)
    {
        output(pos[0], pos[1], 0, 3) = 0;   // set the ccorresponding output pixel as transparent
        counter++;
        // cout << "bg pixel count: " << counter << endl;
        // check neighboring pixels
        dfsMaskBackgroundPixels(Vector2i(pos[0]-1, pos[1]), input, output, visited, bgColor,
            threshold, counter, height, width);
        dfsMaskBackgroundPixels(Vector2i(pos[0], pos[1]-1), input, output, visited, bgColor,
            threshold, counter, height, width);
        dfsMaskBackgroundPixels(Vector2i(pos[0]+1, pos[1]), input, output, visited, bgColor,
            threshold, counter, height, width);
        dfsMaskBackgroundPixels(Vector2i(pos[0], pos[1]+1), input, output, visited, bgColor,
            threshold, counter, height, width);
    }    
}

void dfsBackgroundPixelsWithQueue(const Vector2i& pos, const CImg<unsigned char>& input,
    CImg<unsigned char>& output, CImg<unsigned char>& visited, 
    const Vector3i& bgColor, const double& threshold, int& counter, 
    const int& height, const int& width)
{    
    if ((int)visited(pos[0], pos[1], 0, 0) == 1)
        return;
   
    queue<Vector2i> q;
    q.push(pos);

    while (!q.empty())
    {
        Vector2i cur = q.front();
        q.pop();
        if (cur[0] < 0 || cur[0] >= width || cur[1] < 0 || cur[1] >= height)
            continue;
        if ((int)visited(cur[0], cur[1], 0, 0) == 1)
            continue;
        visited(cur[0], cur[1], 0, 0) = 1;
        
        if (calDiff(input, bgColor, cur) <= threshold)
        {
            output(cur[0], cur[1], 0, 3) = 0;   // set the ccorresponding output pixel as transparent
            output(cur[0], cur[1], 0, 1) = 0;
            output(cur[0], cur[1], 0, 2) = 0;
            output(cur[0], cur[1], 0, 0) = 0;
            counter++;
            // cout << "cur position: " << cur[0] << " " << cur[1] << endl;
            // check neighboring pixels
            q.push(Vector2i(cur[0]-1, cur[1]));
            q.push(Vector2i(cur[0], cur[1]-1));
            q.push(Vector2i(cur[0]+1, cur[1]));
            q.push(Vector2i(cur[0], cur[1]+1));
        }
    }
}

void log(ofstream& fout, string msg, bool silence_flag)
{
    fout << msg << endl;
    if (!silence_flag)
        cout << msg << endl;
}

void extractSil(string inImgName, string outImgName, const Vector3i& bgColor, double threshold)
{
    CImg<unsigned char> img(inImgName.c_str());
    int height = img.height();
    int width = img.width();
    cout << "Loaded image: " << inImgName << endl; 
    cout << "image height: " << height << " image width: " << width << endl;  
  
    // initiliaze a new image with same size
    CImg<unsigned char> output(width, height, 1, 4, 255);
    CImg<unsigned char> visited(width, height, 1, 1, 0);
    // img.display();

    // start from the boundary pixels and expand, mask the pixels as transparent if its
    // color is similar to background color wiithin a threshold
    int bgPixelCount = 0;   // counter for background pixels
    // traverse top and bottom horizontal boundries
    for (int x=0; x<width; x++)
    {
        dfsBackgroundPixelsWithQueue(Vector2i(x, 0), img, output, visited, 
                bgColor, threshold, bgPixelCount, height, width);
        dfsBackgroundPixelsWithQueue(Vector2i(x, height-1), img, output, visited,    
                bgColor, threshold, bgPixelCount, height, width);
    }

    // traverse leftmost and rightmost vertical boundries
    for (int y=0; y<height; y++)
    {
        dfsBackgroundPixelsWithQueue(Vector2i(0, y), img, output, visited, 
            bgColor, threshold, bgPixelCount, height, width);
        dfsBackgroundPixelsWithQueue(Vector2i(width-1, y), img, output, visited, 
            bgColor, threshold, bgPixelCount, height, width);
    }
    
    // log background pixel number 
    ofstream fout("../output/log.txt");
    stringstream s;
    bool silence = false;
    if (bgPixelCount == 0)
    {
        s << "Attention! Image: " << inImgName << " has no background pixels!" << endl;
        log(fout, s.str(), silence);
    }else
    {
        s << inImgName << " has " << bgPixelCount << " background pixels" << endl;
        log(fout, s.str(), silence);
    }
    fout.close();
       

    output.save(outImgName.c_str());
    cout << "Saved image to " << outImgName << endl;
    output.display("output");
}

// Generate visual hull for input images in a row
void batchComputeVisualHull(string inputDir, int resolution)
{
    if (!fs::exists(inputDir))
        cout << "Does not exist path: " << inputDir << "!" << endl;
    vector<string> ObjFileNames;
    getAllFormatFiles(inputDir, ObjFileNames, ".obj");

    for(auto s : ObjFileNames)
    {
        cout << "processing " << s << endl;
        string inMeshName = s;
        // extract the folder path and obj file name from the input path
        string folderPath = s.substr(0, s.find_last_of("/\\"));
        string file = s.substr(s.find_last_of("/\\")+1);
        file = file.substr(0, file.find_last_of("."));
            
        string VHObjName = folderPath + "/" + file + "_vh.obj";        
        // extract the input image name
        vector<string> imgNames;
        for(auto& p: fs::directory_iterator(folderPath))
        {
            if( iequals(p.path().extension(), ".png") )
                imgNames.push_back(p.path());
        }
        
        sort(imgNames.begin(), imgNames.end());
        for (auto s : imgNames)
            cout << s << endl;
        computeVisualHull(imgNames, inMeshName, VHObjName, viewPoints, resolution);
    }    
}


void computeVisualHull(const vector<string>& inImageNames, string inMeshName, string outMeshName, 
        const vector<Vector3d>& viewPoints, int resolution)
{   
    // number of viewpoints must be equal with number of input images
    assert(viewPoints.size() == inImageNames.size());

    // load mesh
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    cout << "Loading " << inMeshName << endl;
	// Read in inputs as double precision floating point meshes
  	igl::readOBJ(inMeshName,V,F);
    const RowVector3d bmin = V.colwise().minCoeff();
	const RowVector3d bmax = V.colwise().maxCoeff();
    RowVector3d center = (bmax + bmin) / 2.0;
	// enlarge bounding box little bit
	RowVector3d Vmin = center + 1.1 * (bmin - center);
	RowVector3d Vmax = center + 1.1 * (bmax - center);

    const double h = (Vmax-Vmin).maxCoeff()/(double)resolution;
	const RowVector3i res = ( resolution * ((Vmax-Vmin) / (Vmax-Vmin).maxCoeff()) ).cast<int>();
    cout << "BBmax: " << Vmax << endl << "BBmin: " << Vmin << endl;
	cout << "Grid resolution: " << res[0] << " " << res[1] << " " << res[2] << endl;

    // generate sampling points
    cout << "generating sampling points ..." << endl;
	MatrixXd GV(res(0)*res(1)*res(2), 3);
    VectorXd S(res(0)*res(1)*res(2), 1);		// S to store visibility values
	for(int zi = 0; zi<res(2); zi++)
	{
		const auto lerp = [&](const int di, const int d)->double
		{return Vmin(d)+(double)di/(double)(res(d)-1)*(Vmax(d)-Vmin(d));};
		const double z = lerp(zi,2);
		for(int yi = 0;yi<res(1);yi++)
		{
			const double y = lerp(yi,1);
			for(int xi = 0;xi<res(0);xi++)
			{
				const double x = lerp(xi,0);
				GV.row(xi+res(0)*(yi + res(1)*zi)) = RowVector3d(x,y,z);
			}
		}
	}

    // project the sampling points to input images -- obtain their screen coordinates
    cout << "computing 2D screen coordinates for each view ..." << endl;
    vector<MatrixXi> screenCoords;
    vector<CImg<unsigned char>> testImages;
    for(int i=0; i<viewPoints.size(); i++)
    {
        eye = viewPoints[i];
        // cout << "Viewpoint " << i << ": " << eye << endl;
        MatrixXi screen =project3DPointsToImage(GV, l, r, b, t, near, far, eye, center, up, img_height, img_width);
        screenCoords.push_back(screen);
        // display projected image - for debug only
        // CImg<unsigned char> testImg(img_width, img_height, 1, 3, 0);
        // for(int j=0; j < screen.rows(); j++)
        // {
        //     int x = screen(j,0);
        //     int y = screen(j,1);
        //     testImg(x, img_height-y, 0) = 255;
        //     testImg(x, img_height-y, 1) = 255;
        //     testImg(x, img_height-y, 2) = 255;
        // }
        // stringstream testImgName;
        // testImgName << "test_" << i << ".png";
        // testImg.save(testImgName.str().c_str());
        // testImages.push_back(testImg);
    }

    vector<CImg<unsigned char>> imgs;
    for(int i=0; i < inImageNames.size(); ++i)
    {
        CImg<unsigned char> curImg(inImageNames[i].c_str());
        imgs.push_back(curImg);
        // cout << inImageNames[i] << endl;
        // curImg.display();
    }

    // compute the visibility of each sampling point
    cout << "checking the visibility of each sampling point ..." << endl;
    // vector<CImg<unsigned char>> tmp = imgs;
    for(int i=0; i < GV.rows(); ++i)
    {
        bool inside = true;
        for(int j=0; j < imgs.size(); j++)
        {
            int x = screenCoords[j](i,0);
            int y = screenCoords[j](i,1);

            // debug only
            // cout << "cur pixel: " << x << " " << y << endl;
            // cout << "current image " << j << endl;
            // CImg<unsigned char> tmp = imgs[j];
            // tmp(x, img_height-y, 0) = 255;
            // tmp(x, img_height-y, 1) = 0;
            // tmp(x, img_height-y, 2) = 0;
            // tmp.display();
           
            if (imgs[j](x, img_height-y, 0)==255 && imgs[j](x, img_height-y, 1)==255 &&
                imgs[j](x, img_height-y, 2)==255)
            {
                inside = false;

                // debug only
                // testImages[j](x, img_height-y, 1) = 0;
                // testImages[j](x, img_height-y, 2) = 0;
                
                break;
            }
        }
        
        // debug only
        // if (inside) cout << "inside" << endl;
        // else cout << "outside" << endl;

        if (inside) S(i) = 0;
        else S(i) = 1;
    }

    // for debug only
    // for(int i=0; i < testImages.size(); i++)
    // {
    //     stringstream testImgName;
    //     testImgName << "test_after_" << i << ".png";
    //     testImages[i].save(testImgName.str().c_str());
    // }

    // use IGL marching cube to reconstruct
	cout << "Extracting surface using marching cubes ... " << endl;
	auto start = high_resolution_clock::now();

	MatrixXd SV;
	MatrixXi SF;
	igl::copyleft::marching_cubes(S, GV, res(0), res(1), res(2), SV, SF);

	auto stop = high_resolution_clock::now(); 
	auto duration = duration_cast<microseconds>(stop - start); 
	cout << "Marching Cube Used time: " << duration.count() / double(1000000.0) << " seconds" << endl;

	// save the reconstructed mesh
	igl::writeOBJ(outMeshName, SV, SF);
	cout << "Finished writing reconstruction to " << outMeshName << "!" << endl;
}

MatrixXi project3DPointsToImage(const MatrixXd& V, float l, float r, float b, float t,
    float near, float far, const Vector3d& eye, const Vector3d& center, const Vector3d& up, int imgHeight, int imgWidth)
{
    // transform to homogeneous coordinates
    // MatrixXd vtemp = V.block(0,0,4,3);
    int numV = V.rows();
    // int numV = vtemp.rows();
    MatrixXd V_homo = MatrixXd::Ones(numV, 4);
    V_homo.block(0,0,numV,3) = V;
    // V_homo.block(0,0,numV,3) = vtemp;
   
    V_homo.transposeInPlace();
    // cout << V_homo << endl;
    // cout << "eye: " << eye << endl;
    OpenGLCamera camera(l, r, b, t, near, far, eye, center, up, imgHeight, imgWidth);
    MatrixXd proj = camera.computeOrthoProjMatrix();
    MatrixXd modelView = camera.computeOrthoModelViewMatrix();
    MatrixXd NDC2Screen = camera.computeOrthoNDC2ScreenMatrix();
    MatrixXd NDC = proj * modelView * V_homo;
    cout << " NDC:" << endl << NDC << endl;
    MatrixXd screen = NDC2Screen * NDC;
    // cout << "proj : " << proj << endl << "modelView: " << modelView << endl << "NDC2Screen: " << NDC2Screen << endl;
    // cout << "NDC: " << endl << NDC << endl;
    // extract the first two rows of screen coordinate matrix to get screen coordinates
    MatrixXi screen2D = screen.block(0,0,2,numV).cast<int>();
    screen2D.transposeInPlace();
    cout << "screen 2d : " << screen2D << endl;

    return screen2D;
}

void projectMeshVertsToImage(string meshName){
    // load mesh
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    cout << "Loading " << meshName << endl;
	// Read in inputs as double precision floating point meshes
  	igl::readOBJ(meshName,V,F);

    MatrixXi screen2D = project3DPointsToImage(V, l, r, b, t, near, far, eye, center, up, img_height, img_width);
    
    // project the points on the image
    CImg<unsigned char> image(imgName.c_str());
    cout << "alpha: " << image(0,0,0,3) << endl;
    // system("pause");
    for(int i=0; i < screen2D.rows(); ++i)
    {
        int x = screen2D(i,0);
        int y = screen2D(i,1);
        // cout << x << " " << y << endl;
        image( x, img_height - y, 0) = 255;
        image( x, img_height - y, 1) = 255;
        image( x, img_height - y, 2) = 255;
        // cout << "depth: " << image( x, img_height - y, 0, 3) << endl;
    }
    image.display("Projection");
    // return screen2D;
}