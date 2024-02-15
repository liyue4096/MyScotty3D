#include "framebuffer.h"
#include "../util/hdr_image.h"
#include "sample_pattern.h"

Framebuffer::Framebuffer(uint32_t width_, uint32_t height_, SamplePattern const& sample_pattern_)
	: width(width_), height(height_), sample_pattern(sample_pattern_) {

	// check that framebuffer isn't larger than allowed:
	if (width > MaxWidth || height > MaxHeight) {
		throw std::runtime_error("Framebuffer size (" + std::to_string(width) + "x" +
		                         std::to_string(height) + ") exceeds maximum allowed (" +
		                         std::to_string(MaxWidth) + "x" + std::to_string(MaxHeight) + ").");
	}
	// check that framebuffer size is even:
	if (width % 2 != 0 || height % 2 != 0) {
		throw std::runtime_error("Framebuffer size (" + std::to_string(width) + "x" +
		                         std::to_string(height) + ") is not even.");
	}

	uint32_t samples =
		width * height * static_cast<uint32_t>(sample_pattern.centers_and_weights.size());
	//info("Framebuffer samples size = %d * %d * %d\n", width, height, sample_pattern.centers_and_weights.size());
	// allocate storage for color and depth samples:
	colors.assign(samples, Spectrum{0.0f, 0.0f, 0.0f});
	depths.assign(samples, 1.0f);
}

HDR_Image Framebuffer::resolve_colors() const {
	// A1T7: resolve_colors
	// TODO: update to support sample patterns with more than one sample.

	HDR_Image image(width, height);
	//std::vector<Vec3> centers_and_weights;
	//std::string name = "Custom Sample Pattern";
	//SamplePattern sp((uint32_t)0, name, centers_and_weights);
	//float weight = 1.0f / centers_and_weights.size();

	int size = (int)this->sample_pattern.centers_and_weights.size();
	//info("\n size: %d \n", size);
	//retrieve weights
	std::vector<float> weigths(size, 0);
	for (int i = 0; i < size; i++)
	{
		weigths[i] = sample_pattern.centers_and_weights[i].z;
		//info("\n weights: %d: %f \n", i, weigths[i]);
	}
	
	for (uint32_t y = 0; y < height; ++y) {
		for (uint32_t x = 0; x < width; ++x) {
			for(int i = 0; i < size; ++i){
				image.at(x, y) += color_at(x, y, i) * weigths[i];
				//if( i == 0 && x == 150)
					//info("\n local image.at(%d, %d, 0): %f, %f, %f", int(x), int(y), color_at(x, y, i).r, color_at(x, y, i).g, color_at(x, y, i).b);
			}
			//if (x  == (uint32_t)width/2 )
			{
				//info("\n image.at(%d, %d): %f, %f, %f", int(x), int(y), image.at(x, y).r, image.at(x, y).g, image.at(x, y).b);
			}		
		}
	}

	return image;
}
