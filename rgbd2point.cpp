#include <cstdio>
#include <OpenNI.h>
#include <cmath>

namespace rgbd2point {
	const int read_wait_timeout = 100;
	
	const int depth_averaging_threshold = 300;
}	

struct RawData {
	int resx;
	int resy;
	
	int *r;
	int *g;
	int *b;
	int clrframenum;
	
	long *d;
	int *dframenums;
	
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

void init_rawdata(RawData *raw, int resx, int resy) {
	raw->resx = resx;
	raw->resy = resy;
	
	raw->r = new int[resx*resy];
	raw->g = new int[resx*resy];
	raw->b = new int[resx*resy];
	raw->d = new long[resx*resy];
	raw->dframenums = new int[resx*resy];
	
	for(int y = 0; y < resy; y++) {
		for(int x = 0; x < resx; x++) {
			raw->r[x+y*resx] = 0;
			raw->g[x+y*resx] = 0;
			raw->b[x+y*resx] = 0;
			raw->d[x+y*resx] = 0;
			raw->dframenums[x+y*resx] = 0;
		}
	}
	
	raw->clrframenum = 0;
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
	
	if(frame.getVideoMode().getResolutionX() != data.resx || frame.getVideoMode().getResolutionY() != data.resy) {
		printf("Unexpected resolution change (%dx%d -> %dx%d)\n", data.resx, data.resy,
			   frame.getVideoMode().getResolutionX(), frame.getVideoMode().getResolutionY());
		
		exit(3);
	}
	
	int takeframe = 0;
	
	switch (frame.getVideoMode().getPixelFormat())
	{
	case openni::PIXEL_FORMAT_DEPTH_1_MM:
	case openni::PIXEL_FORMAT_DEPTH_100_UM:
		depthpix = (openni::DepthPixel*)frame.getData();
		for(y = 0; y < data.resy; y++) {
			for(x = 0; x < data.resx; x++) {
				float curavg = data.d[x+data.resx*y]/(float)data.dframenums[x+data.resx*y];
				
				if(depthpix[x+data.resx*y] == 0)
					continue;
				
				if(data.d[x+data.resx*y] == 0 || fabs(curavg-depthpix[x+data.resx*y]) < rgbd2point::depth_averaging_threshold) {
					data.d[x+data.resx*y] += depthpix[x+data.resx*y];
					data.dframenums[x+data.resx*y]++;
				}		
				
			}
		}
		break;
	case openni::PIXEL_FORMAT_RGB888:
		clrpix = (openni::RGB888Pixel*)frame.getData();
		for(y = 0; y < data.resy; y++) {
			for(x = 0; x < data.resx; x++) {
				data.r[x+data.resx*y] += clrpix[x+data.resx*y].r;
				data.g[x+data.resx*y] += clrpix[x+data.resx*y].g;
				data.b[x+data.resx*y] += clrpix[x+data.resx*y].b;		
				
				
			}
		}
		
		data.clrframenum++;
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
	
	for(y = 0; y < raw.resy; y++) {
		for(x = 0; x < raw.resx; x++) {
			if(raw.dframenums[x+y*raw.resx] == 0)
				continue;
			
			avgdepth = raw.d[x+y*raw.resx]/(float)raw.dframenums[x+y*raw.resx];
							
			openni::CoordinateConverter::convertDepthToWorld(depthstrm, (float)x, (float)y, avgdepth, &cloud.x[i], &cloud.y[i], &cloud.z[i]);
						
			cloud.r[i] = raw.r[x+y*raw.resx]/(float)raw.clrframenum;
			cloud.g[i] = raw.g[x+y*raw.resx]/(float)raw.clrframenum;
			cloud.b[i] = raw.b[x+y*raw.resx]/(float)raw.clrframenum;
			i++;
		}
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
	init_rawdata(&raw, depth.getVideoMode().getResolutionX(), depth.getVideoMode().getResolutionY());
				 
	
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
	init_pointcloud(&cloud, raw.resx*raw.resy);
	depth_to_pointcloud(cloud, raw, depth, color);
	export_to_ply(argv[2], cloud);
	
	free_rawdata(&raw);
	free_pointcloud(&cloud);
	depth.stop();
	color.stop();
	depth.destroy();
	color.destroy();
	device.close();
	openni::OpenNI::shutdown();
}