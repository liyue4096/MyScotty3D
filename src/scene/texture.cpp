
#include "texture.h"

#include <iostream>

namespace Textures {


Spectrum sample_nearest(HDR_Image const &image, Vec2 uv) {
	//clamp texture coordinates, convert to [0,w]x[0,h] pixel space:
	float x = image.w * std::clamp(uv.x, 0.0f, 1.0f);
	float y = image.h * std::clamp(uv.y, 0.0f, 1.0f);

	//the pixel with the nearest center is the pixel that contains (x,y):
	int32_t ix = int32_t(std::floor(x));
	int32_t iy = int32_t(std::floor(y));

	//texture coordinates of (1,1) map to (w,h), and need to be reduced:
	ix = std::min(ix, int32_t(image.w) - 1);
	iy = std::min(iy, int32_t(image.h) - 1);

	return image.at(ix, iy);
}

Spectrum sample_bilinear(HDR_Image const &image, Vec2 uv) {
	// A1T6: sample_bilinear
	//TODO: implement bilinear sampling strategy on texture 'image'
	//clamp texture coordinates, convert to [0,w]x[0,h] pixel space:
	float x = image.w * std::clamp(uv.x, 0.0f, 1.0f) - 0.5f;
	float y = image.h * std::clamp(uv.y, 0.0f, 1.0f) - 0.5f;

	//the pixel with the nearest center is the pixel that contains (x,y):
	int32_t ix_0 = int32_t(std::clamp((float)std::floor(x), 0.f, float(image.w) - 1));
	int32_t iy_0 = int32_t(std::clamp((float)std::floor(y), 0.f, float(image.h) - 1));

	//std::cout << "ix_0: " << ix_0 << "\n" ;

	int32_t ix_1 = int32_t(std::clamp((float)std::floor(x)+1, 0.f, float(image.w) - 1));
	int32_t iy_1 = int32_t(std::clamp((float)std::floor(y)+1, 0.f, float(image.h) - 1));

	//double int_part;
	float alpha = x - ix_0;
	float beta = y - iy_0;

	Spectrum outputSpec;

	outputSpec = (1-alpha)*(1-beta)	* image.at(ix_0, iy_0) + alpha*(1-beta)	* image.at(ix_1, iy_0) + 
				(1-alpha)*beta * image.at(ix_0, iy_1) + alpha*beta * image.at(ix_1, iy_1);

	return outputSpec; //placeholder so image doesn't look blank
}

static int static_level = -9, fragment_cnt = 0;
Spectrum sample_trilinear(HDR_Image const &base, std::vector< HDR_Image > const &levels, Vec2 uv, float lod) {
	// A1T6: sample_trilinear
	//TODO: implement trilinear sampling strategy on using mip-map 'levels'
	int d1, d2;
	d1 = (int)std::floor(lod); //std::clamp((int)std::floor(lod-1), -1, (int)levels.size()-1);
	d2 = (int)std::ceil(lod); //std::clamp((int)std::floor(lod-1)+1, -1, (int)levels.size()-1);
	//d2 = (int)std::clamp((float)d1, 0.f, (float)(levels.size()) - 1);
	Spectrum outputSpec;
	if (d2 > (int)levels.size())
	{
		return sample_bilinear(levels.at((int)levels.size() - 1), uv);
	}
	if (d2 < 0)
	{
		d2 = 0;
	}
	
	float frac = lod - d1;
	// std::cout<<frac << "\n";

	fragment_cnt++;
	//test 
	if (d1 > static_level)
	{
		static_level = d1;
		// std::cout << "levels.size():" << levels.size() << ", lod: " << lod << ", d2: " << d2 << ", frac: " << frac << ", fragment_cnt: " << fragment_cnt 
		// 	<< ", coor: " << uv.x << ", " << uv.y <<"\n";
	}
	if (d1 < 0)
	{
		outputSpec = sample_bilinear(base, uv);
	}
	else if (d1 == 0)
	{

		outputSpec = (1-frac)*sample_bilinear(base, uv) + frac * sample_bilinear(levels.at(0), uv);
	}	
	else
	{
		outputSpec = (1-frac)*sample_bilinear(levels.at(d1-1), uv) + frac * sample_bilinear(levels.at(d2-1), uv);
	}	

	return outputSpec; //placeholder so image doesn't look blank
}

/*
 * generate_mipmap- generate mipmap levels from a base image.
 *  base: the base image
 *  levels: pointer to vector of levels to fill (must not be null)
 *
 * generates a stack of levels [1,n] of sizes w_i, h_i, where:
 *   w_i = max(1, floor(w_{i-1})/2)
 *   h_i = max(1, floor(h_{i-1})/2)
 *  with:
 *   w_0 = base.w
 *   h_0 = base.h
 *  and n is the smalles n such that w_n = h_n = 1
 *
 * each level should be calculated by downsampling a blurred version
 * of the previous level to remove high-frequency detail.
 *
 */
void generate_mipmap(HDR_Image const &base, std::vector< HDR_Image > *levels_) {
	assert(levels_);
	auto &levels = *levels_;


	{ // allocate sublevels sufficient to scale base image all the way to 1x1:
		int32_t num_levels = static_cast<int32_t>(std::log2(std::max(base.w, base.h)));
		assert(num_levels >= 0);

		levels.clear();
		levels.reserve(num_levels);

		uint32_t width = base.w;
		uint32_t height = base.h;
		for (int32_t i = 0; i < num_levels; ++i) {
			assert(!(width == 1 && height == 1)); //would have stopped before this if num_levels was computed correctly

			width = std::max(1u, width / 2u);
			height = std::max(1u, height / 2u);

			levels.emplace_back(width, height);
		}
		assert(width == 1 && height == 1);
		assert(levels.size() == uint32_t(num_levels));
	}

	//now fill in the levels using a helper:
	//downsample:
	// fill in dst to represent the low-frequency component of src
	auto downsample = [](HDR_Image const &src, HDR_Image &dst) {
		//dst is half the size of src in each dimension:
		assert(std::max(1u, src.w / 2u) == dst.w);
		assert(std::max(1u, src.h / 2u) == dst.h);

		// A1T6: generate
		//TODO: Write code to fill the levels of the mipmap hierarchy by downsampling
		if (src.w == 1 && src.h == 1)
		{
			return;
		}
		for (size_t i = 0; i < dst.w; i++)
		{
			for (size_t j = 0; j < dst.h; j++)
			{
				Vec2 uv((i+0.5f)/dst.w, (j+0.5f)/dst.h);
				dst.at((int)i, (int)j) = sample_bilinear(src, uv);
			}			
		}
		return; 
		//Be aware that the alignment of the samples in dst and src will be different depending on whether the image is even or odd.
		uint32_t pixel_size = dst.w * dst.h, ref;
		std::vector<Spectrum> pixels(pixel_size, Spectrum());
		Spectrum local_spec;
		
		if (src.w > 1 && src.h > 1)
		{
			for (int i = 0; i < (int)dst.h; i++)
			{
				for (int j = 0; j < (int)dst.w; j++)
				{
					//offset = 2 * (dst.w % 2) * i;
					ref = 2 * src.w * i + 2 * j;
					local_spec = (src.at(ref) + src.at(ref+1) + src.at(ref+src.w) + src.at(ref+src.w+1)) / 4.f;	
					pixels[i*dst.w + j] = local_spec;			
				}				
			}			
		}
		else if ((int)src.w == 1 || (int)src.h == 1)
		{
			for (int j = 0; j < std::max((int)src.w, (int)src.h); j++)
			{
				ref = 2 * (uint32_t)j;
				local_spec = (src.at(ref) + src.at(ref+1)) / 2.f;
				pixels[j] = local_spec;
			} 
		}
		//std::cout << "here!!!!!!!!!!: dst.w & h" << dst.w << ", " << dst.h << "\n";
		dst = HDR_Image(dst.w, dst.h, pixels);
	};

	std::cout << "Regenerating mipmap (" << levels.size() << " levels): [" << base.w << "x" << base.h << "]";
	std::cout.flush();
	for (uint32_t i = 0; i < levels.size(); ++i) {
		HDR_Image const &src = (i == 0 ? base : levels[i-1]);
		HDR_Image &dst = levels[i];
		std::cout << " -> [" << dst.w << "x" << dst.h << "]"; std::cout.flush();

		downsample(src, dst);
	}
	std::cout << std::endl;
	
}

Image::Image(Sampler sampler_, HDR_Image const &image_) {
	sampler = sampler_;
	image = image_.copy();
	update_mipmap();
}

Spectrum Image::evaluate(Vec2 uv, float lod) const {
	if (image.w == 0 && image.h == 0) return Spectrum();
	if (sampler == Sampler::nearest) {
		return sample_nearest(image, uv);
	} else if (sampler == Sampler::bilinear) {
		return sample_bilinear(image, uv);
	} else {
		//std::cout << "\n evalutate: lod = " << lod << "\n";
		return sample_trilinear(image, levels, uv, lod);
	}
}

void Image::update_mipmap() {
	if (sampler == Sampler::trilinear) {
		generate_mipmap(image, &levels);
	} else {
		levels.clear();
	}
}

GL::Tex2D Image::to_gl() const {
	return image.to_gl(1.0f);
}

void Image::make_valid() {
	update_mipmap();
}

Spectrum Constant::evaluate(Vec2 uv, float lod) const {
	return color * scale;
}

} // namespace Textures
bool operator!=(const Textures::Constant& a, const Textures::Constant& b) {
	return a.color != b.color || a.scale != b.scale;
}

bool operator!=(const Textures::Image& a, const Textures::Image& b) {
	return a.image != b.image;
}

bool operator!=(const Texture& a, const Texture& b) {
	if (a.texture.index() != b.texture.index()) return false;
	return std::visit(
		[&](const auto& data) { return data != std::get<std::decay_t<decltype(data)>>(b.texture); },
		a.texture);
}
