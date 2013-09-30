#include <cstdio>
#include <OpenNI.h>
#include <cmath>

namespace rgbd2point {
	const int read_wait_timeout = 100;
	
	const int depth_averaging_threshold = 300;
}	

struct RawData {
	// Depth
	
	int dresx;
	int dresy;
	
	long *d;
	int *dframenums; // some pixels are rejected for the average so
					 // the number of frames is pixel dependent
	
	// Color
	
	int cresx; 
	int cresy; 
	
	int *r;
	int *g;
	int *b;
	int cframenum;	
};

struct PointCloud {
	float *x;
	float *y;
	float *z;
	
	uint8_t *r;
	uint8_t *g;
	uint8_t *b;
	
	int num;
};

void init_rawdata(RawData *raw, int dresx, int dresy, int cresx, int cresy) {
	raw->dresx = dresx;
	raw->dresy = dresy;
	
	raw->cresx = cresx;
	raw->cresy = cresy;
	
	raw->r = new int[cresx*cresy];
	raw->g = new int[cresx*cresy];
	raw->b = new int[cresx*cresy];
	
	raw->d = new long[dresx*dresy];
	raw->dframenums = new int[dresx*dresy];
	
	for(int y = 0; y < cresy; y++) {
		for(int x = 0; x < cresx; x++) {
			raw->r[x+y*cresx] = 0;
			raw->g[x+y*cresx] = 0;
			raw->b[x+y*cresx] = 0;			
		}
	}
	
	for(int y = 0; y < dresy; y++) {
		for(int x = 0; x < dresx; x++) {
			raw->d[x+y*dresx] = 0;
			raw->dframenums[x+y*dresx] = 0;
		}
	}
	
	raw->cframenum = 0;
}

void init_pointcloud(PointCloud *cloud, int num) {
	cloud->num = num;
	
	cloud->x = new float[num];
	cloud->y = new float[num];
	cloud->z = new float[num];
	cloud->r = new uint8_t[num];
	cloud->g = new uint8_t[num];
	cloud->b = new uint8_t[num];
		
}

void free_rawdata(RawData *raw) {
	delete[] raw->r;
	delete[] raw->g;
	delete[] raw->b;
	delete[] raw->d;
	delete[] raw->dframenums;
}

void free_pointcloud(PointCloud *cloud) {
	delete[] cloud->x;
	delete[] cloud->y;
	delete[] cloud->z;
	delete[] cloud->r;
	delete[] cloud->g;
	delete[] cloud->b;	
}

void init_openni(openni::Device &device, char *onifile) {
	openni::Status rc = openni::OpenNI::initialize();
	if(rc != openni::STATUS_OK)	{
		printf("Initialize failed\n%s\n", openni::OpenNI::getExtendedError());
		exit(1);
	}

	
	rc = device.open(onifile);
	if(rc != openni::STATUS_OK) {
		printf("Couldn't open device\n%s\n", openni::OpenNI::getExtendedError());
		exit(2);
	}
}

void read_frame(openni::VideoFrameRef &frame, RawData &data) {
	openni::DepthPixel *depthpix;
	openni::RGB888Pixel *clrpix;
	int x, y;
	
		
	int takeframe = 0;
	
	switch (frame.getVideoMode().getPixelFormat()) {
	case openni::PIXEL_FORMAT_DEPTH_1_MM:
	case openni::PIXEL_FORMAT_DEPTH_100_UM:
		depthpix = (openni::DepthPixel*)frame.getData();
		for(y = 0; y < data.dresy; y++) {
			for(x = 0; x < data.dresx; x++) {
				float curavg = data.d[x+data.dresx*y]/(float)data.dframenums[x+data.dresx*y];
				
				if(depthpix[x+data.dresx*y] == 0)
					continue;
				
				if(data.d[x+data.dresx*y] == 0 || fabs(curavg-depthpix[x+data.dresx*y]) < rgbd2point::depth_averaging_threshold) {
					data.d[x+data.dresx*y] += depthpix[x+data.dresx*y];
					data.dframenums[x+data.dresx*y]++;
				}		
				
			}
		}
		break;
	case openni::PIXEL_FORMAT_RGB888:
		clrpix = (openni::RGB888Pixel*)frame.getData();
		for(y = 0; y < data.cresy; y++) {
			for(x = 0; x < data.cresx; x++) {
				data.r[x+data.cresx*y] += clrpix[x+data.cresx*y].r;
				data.g[x+data.cresx*y] += clrpix[x+data.cresx*y].g;
				data.b[x+data.cresx*y] += clrpix[x+data.cresx*y].b;		
				
				
			}
		}
		
		data.cframenum++;
		break;
	default:
		printf("Unknown format\n");
	}	
	
}

void depth_to_pointcloud(PointCloud &cloud, RawData &raw, openni::VideoStream &depthstrm, openni::VideoStream &clrstrm) {
	int x, y;
	
	float avgdepth;
	int clrx;
	int clry;
	int i = 0;
	
	for(y = 0; y < raw.dresy; y++) {
		for(x = 0; x < raw.dresx; x++) {
			if(raw.dframenums[x+y*raw.dresx] == 0)
				continue;
			
			avgdepth = raw.d[x+y*raw.dresx]/(float)raw.dframenums[x+y*raw.dresx];
							
			openni::CoordinateConverter::convertDepthToWorld(depthstrm, (float)x, (float)y, avgdepth, &cloud.x[i], &cloud.y[i], &cloud.z[i]);
			
			int cx = x/(float)raw.dresx*raw.cresx;
			int cy = y/(float)raw.dresy*raw.cresy;
			
			if(cx >= raw.cresx)
				cx--;
			if(cy >= raw.cresy)
				cy--;
			
			cloud.r[i] = raw.r[cx+cy*raw.cresx]/(float)raw.cframenum;
			cloud.g[i] = raw.g[cx+cy*raw.cresx]/(float)raw.cframenum;
			cloud.b[i] = raw.b[cx+cy*raw.cresx]/(float)raw.cframenum;
			i++;
			
// 			cloud.z[i]*=-1;
		}
		
		printf("\r%.1f%%", y/(float)(raw.dresy-1)*100.0);
	}
	
	cloud.num = i;
}

void export_to_ply(char *filename, PointCloud &c) {
	FILE *f = fopen(filename, "wb");
	
	fprintf(f, "ply\n"
			   "format ascii 1.0\n"
			   "comment created by rgbdsend\n"
			   "element vertex %d\n"
			   "property float32 x\n"
			   "property float32 y\n"
			   "property float32 z\n"
			   "property uint8 red\n"
			   "property uint8 green\n"
			   "property uint8 blue\n"
			   "element face 0\n"
			   "property list uint8 int32 vertex_indices\n"
			   "end_header\n", c.num);
	
	for(int i = 0; i < c.num; i++)
		fprintf(f, "%f %f %f %d %d %d\n", c.x[i], c.y[i], c.z[i], c.r[i], c.g[i], c.b[i]);
	
	fclose(f);
}	

int main(int argc, char **argv) {
	openni::Device device;
	openni::Status rc;
	
	if(argc != 3) {
		printf("Usage: %s ONIFILE OUTFILE\n", argv[0]);
		exit(1);
	}
		
	init_openni(device, argv[1]);
	
	openni::VideoStream depth, color;
		
	if(device.getSensorInfo(openni::SENSOR_DEPTH) != NULL) {
		rc = depth.create(device, openni::SENSOR_DEPTH);
		if(rc == openni::STATUS_OK)	{
			rc = depth.start();
			if(rc != openni::STATUS_OK)	{
				printf("Couldn't start the color stream\n%s\n", openni::OpenNI::getExtendedError());
			}
		}
		else {
			printf("Couldn't create depth stream\n%s\n", openni::OpenNI::getExtendedError());
		}
	}

	if(device.getSensorInfo(openni::SENSOR_COLOR) != NULL) {
		rc = color.create(device, openni::SENSOR_COLOR);
		if(rc == openni::STATUS_OK)	{
			rc = color.start();
			if(rc != openni::STATUS_OK)	{
				printf("Couldn't start the color stream\n%s\n", openni::OpenNI::getExtendedError());
			}
		}
		else {
			printf("Couldn't create color stream\n%s\n", openni::OpenNI::getExtendedError());
		}
	}
	
	RawData raw;
	init_rawdata(&raw, depth.getVideoMode().getResolutionX(), depth.getVideoMode().getResolutionY(), color.getVideoMode().getResolutionX(), color.getVideoMode().getResolutionY());
				 
	
	openni::VideoFrameRef frame;

	openni::VideoStream* streams[] = {&depth, &color};
	
	device.getPlaybackControl()->setRepeatEnabled(false);
	
	while(1) {
		int readyStream = -1;
		rc = openni::OpenNI::waitForAnyStream(streams, 2, &readyStream, rgbd2point::read_wait_timeout);
		if (rc != openni::STATUS_OK) {
			printf("Finished reading recording.\n");
			break;
		}

		switch (readyStream) {
		case 0:
			// Depth
			depth.readFrame(&frame);
			break;
		case 1:
			// Color
			color.readFrame(&frame);
			break;
		default:
			printf("Unexpected stream\n");
		}	
		
		read_frame(frame, raw);
	}
		
	PointCloud cloud;
	init_pointcloud(&cloud, raw.dresx*raw.dresy);
	depth_to_pointcloud(cloud, raw, depth, color);
	export_to_ply(argv[2], cloud);
	
	printf("\nExtracted to point cloud.\n");
	
	free_rawdata(&raw);
	free_pointcloud(&cloud);
	depth.stop();
	color.stop();
	depth.destroy();
	color.destroy();
	device.close();
	openni::OpenNI::shutdown();
}