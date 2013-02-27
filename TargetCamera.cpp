#include "TargetCamera.h"
#include "ImageProcessing.h"

TargetCamera::TargetCamera (void)
{
	AxisCamera &targetCamera =AxisCamera::GetInstance("10.24.74.11");
	lastDistance=0;
	targetTime=0;
	refreshRate=100000;
	debugMode=false;
	
	
}

float TargetCamera::GetLastDistance(void)
{
	return lastDistance;
}

float TargetCamera::GetDistance(void)
{
	if (targetTime <= GetFPGATime())
	{
		if (ComputeDistance() == 0)
		{
			targetTime=GetFPGATime()+refreshRate;
		}
		else
		{
			//TODO handle the case where no target was found
		}
	}
	return lastDistance;
}

bool TargetCamera::GetDebugMode (void)
{
	return debugMode;
}

void TargetCamera::SetDebugMode(bool debug)
{
	debugMode = debug;
}

unsigned long TargetCamera::GetRefreshRate(void)
{
	return refreshRate;
}

void TargetCamera::SetRefreshRate(unsigned long rate)
{
	refreshRate = rate;
}

int TargetCamera::ComputeDistance(void)
{
	int returnVal = -1; 
	
	AxisCamera &targetCamera =AxisCamera::GetInstance("10.24.74.11");
	ColorImage *colorImage = new ColorImage(IMAQ_IMAGE_HSL);
	BinaryImage *binaryImage;
	targetCamera.GetImage(colorImage);
	Image *imaqImage;
	if ((colorImage == (void *) 0) || 
		(colorImage->GetWidth() <= 0) || (colorImage->GetWidth() > 1000) ||
		(colorImage->GetHeight() <= 0) || (colorImage->GetHeight() > 1000))
	{
	    delete colorImage;
		return returnVal;
	}
	
	if (debugMode==true)
	{
		colorImage->Write("capturedImage.jpg");
	}
	
	//binaryImage = colorImage->ThresholdHSL(90, 115,30, 255, 70, 255);
	binaryImage = colorImage->ThresholdHSL(80, 135,70, 255, 90, 255);
		
	if ((binaryImage == (void *) 0) || (binaryImage->GetWidth() != 320) || (binaryImage->GetHeight() != 240))
	{
	    delete colorImage;
	    delete binaryImage;
		return returnVal;
	}
	
	imaqImage = binaryImage->GetImaqImage();
	
	if (imaqImage == (void *) 0)
	{
	    delete colorImage;
	    delete binaryImage;
		return returnVal;
	}
	
	if (debugMode==true)
	{
		binaryImage->Write("afterCLRThreshold.bmp");
		IVA_ProcessImage(imaqImage, binaryImage);
	}	
	else
	{
		IVA_ProcessImage(imaqImage, (ImageBase *)0);
	}
	vector<ParticleAnalysisReport> *reports = binaryImage->GetOrderedParticleAnalysisReports();
	ParticleAnalysisReport *topParticlePtr = NULL;
	
	for (unsigned int particleIndex = 0; particleIndex < reports->size(); particleIndex++)
	{
		ParticleAnalysisReport &thisReport = reports->at(particleIndex);
		//TODO Validate the particle so we don't end up targeting a spec on the wall? (particle filter gets rid fo really small stuff, but we may want this if the arena is noisy)
		if ((!topParticlePtr) || (thisReport.center_mass_y  < topParticlePtr->center_mass_y)  /*&& thisReport.particleQuality > WHATT!!!!*/)
		{
			topParticlePtr = &thisReport;
		}
	}
	
	if (topParticlePtr != NULL)
	{
		float target_height_pixels = topParticlePtr->boundingRect.height;
		const float target_height_inches = 1.66666;
		const float half_vertical_resolution = 120;
		const float tangent_of_vertical_field_of_view = .347007;
		lastDistance = (target_height_inches*half_vertical_resolution*tangent_of_vertical_field_of_view)/target_height_pixels;
		returnVal = 0;
	}
	delete reports;
	delete binaryImage;
    delete colorImage;
	return returnVal;
}
