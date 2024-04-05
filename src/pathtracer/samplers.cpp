
#include "samplers.h"
#include "../util/rand.h"
#include "../scene/shape.h"

constexpr bool IMPORTANCE_SAMPLING = true;

namespace Samplers
{

	Vec2 Rect::sample(RNG &rng) const
	{
		// A3T1 - step 2 - supersampling

		// Return a point selected uniformly at random from the rectangle [0,size.x)x[0,size.y)
		// Useful function: rng.unit()
		float x = rng.unit() * size.x;
		float y = rng.unit() * size.y;

		return Vec2{x, y};
	}

	float Rect::pdf(Vec2 at) const
	{
		if (at.x < 0.0f || at.x > size.x || at.y < 0.0f || at.y > size.y)
			return 0.0f;
		return 1.0f / (size.x * size.y);
	}

	Vec2 Circle::sample(RNG &rng) const
	{
		// A3EC - bokeh - circle sampling

		// Return a point selected uniformly at random from a circle defined by its
		// center and radius.
		// Useful function: rng.unit()

		return Vec2{};
	}

	float Circle::pdf(Vec2 at) const
	{
		// A3EC - bokeh - circle pdf

		// Return the pdf of sampling the point 'at' for a circle defined by its
		// center and radius.

		return 1.f;
	}

	Vec3 Point::sample(RNG &rng) const
	{
		return point;
	}

	float Point::pdf(Vec3 at) const
	{
		return at == point ? 1.0f : 0.0f;
	}

	Vec3 Triangle::sample(RNG &rng) const
	{
		float u = std::sqrt(rng.unit());
		float v = rng.unit();
		float a = u * (1.0f - v);
		float b = u * v;
		return a * v0 + b * v1 + (1.0f - a - b) * v2;
	}

	float Triangle::pdf(Vec3 at) const
	{
		float a = 0.5f * cross(v1 - v0, v2 - v0).norm();
		float u = 0.5f * cross(at - v1, at - v2).norm() / a;
		float v = 0.5f * cross(at - v2, at - v0).norm() / a;
		float w = 1.0f - u - v;
		if (u < 0.0f || v < 0.0f || w < 0.0f)
			return 0.0f;
		if (u > 1.0f || v > 1.0f || w > 1.0f)
			return 0.0f;
		return 1.0f / a;
	}

	Vec3 Hemisphere::Uniform::sample(RNG &rng) const
	{

		float Xi1 = rng.unit();
		float Xi2 = rng.unit();

		float theta = std::acos(Xi1);
		float phi = 2.0f * PI_F * Xi2;

		float xs = std::sin(theta) * std::cos(phi);
		float ys = std::cos(theta);
		float zs = std::sin(theta) * std::sin(phi);

		return Vec3(xs, ys, zs);
	}

	float Hemisphere::Uniform::pdf(Vec3 dir) const
	{
		if (dir.y < 0.0f)
			return 0.0f;
		return 1.0f / (2.0f * PI_F);
	}

	Vec3 Hemisphere::Cosine::sample(RNG &rng) const
	{

		float phi = rng.unit() * 2.0f * PI_F;
		float cos_t = std::sqrt(rng.unit());

		float sin_t = std::sqrt(1 - cos_t * cos_t);
		float x = std::cos(phi) * sin_t;
		float z = std::sin(phi) * sin_t;
		float y = cos_t;

		return Vec3(x, y, z);
	}

	float Hemisphere::Cosine::pdf(Vec3 dir) const
	{
		if (dir.y < 0.0f)
			return 0.0f;
		return dir.y / PI_F;
	}

	Vec3 Sphere::Uniform::sample(RNG &rng) const
	{
		// A3T7 - sphere sampler

		// Generate a uniformly random point on the unit sphere.
		// Tip: start with Hemisphere::Uniform
		Hemisphere::Uniform hemi_uniform_sampler;
		Vec3 coordinate;
		if (rng.unit() >= 0.5)
		{
			return hemi_uniform_sampler.sample(rng);
		}
		else
			coordinate = hemi_uniform_sampler.sample(rng);
		coordinate.y = -coordinate.y;
		return coordinate;
	}

	float Sphere::Uniform::pdf(Vec3 dir) const
	{
		return 1.0f / (4.0f * PI_F);
	}

	Sphere::Image::Image(const HDR_Image &image)
	{
		// A3T7 - image sampler init

		// Set up importance sampling data structures for a spherical environment map image.
		// You may make use of the _pdf, _cdf, and total members, or create your own.

		const auto [_w, _h] = image.dimension();
		w = _w;
		h = _h;

		int index = 0;
		float tmp_luma = 0.f;
		float local_luma = 0.f;
		for (auto pixel : image.data())
		{
			local_luma = pixel.luma();
			int y = index / w;
			float v = (y + 0.5f) / h;
			float theta = PI_F - v * PI_F;
			// void(sin(theta));
			_cdf.emplace_back(tmp_luma + local_luma * sin(theta));
			tmp_luma = _cdf.back();
			index++;
		}

		// normalizaiton:
		index = 0;
		for (auto &value : _cdf)
		{
			value /= tmp_luma;
		}

		_pdf.resize(_cdf.size());
		if (!_cdf.empty())
		{
			_pdf[0] = _cdf[0];
			for (size_t i = 1; i < _cdf.size(); ++i)
			{
				_pdf[i] = _cdf[i] - _cdf[i - 1];
			}
		}
	}

	Vec3 Sphere::Image::sample(RNG &rng) const
	{
		if (!IMPORTANCE_SAMPLING)
		{
			// Step 1: Uniform sampling
			// Declare a uniform sampler and return its sample
			Sphere::Uniform uniform_sampler;
			auto v3 = uniform_sampler.sample(rng);
			return v3;
		}
		else
		{
			// Step 2: Importance sampling
			// Use your importance sampling data structure to generate a sample direction.
			// Tip: std::upper_bound

			auto it = std::upper_bound(_cdf.begin(), _cdf.end(), rng.unit());
			int index = (int)std::distance(_cdf.begin(), it);

			int y = index / w;
			int x = index % w;

			float u = (x + 0.5f) / w;
			float v = (y + 0.5f) / h;

			float phi = u * 2.0f * PI_F;
			float theta = PI_F - v * PI_F;

			Vec3 direction;
			direction.x = std::sin(theta) * std::cos(phi);
			direction.y = std::cos(theta);
			direction.z = std::sin(theta) * std::sin(phi);

			return direction;
		}
	}

	float Sphere::Image::pdf(Vec3 dir) const
	{
		if (!IMPORTANCE_SAMPLING)
		{
			// Step 1: Uniform sampling
			// Declare a uniform sampler and return its pdf
			Sphere::Uniform uniform_sample;
			return uniform_sample.pdf(dir);
		}
		else
		{
			// A3T7 - image sampler importance sampling pdf
			// What is the PDF of this distribution at a particular direction?

			Vec2 uv = Shapes::Sphere().uv(dir.unit());
			float sin_theta = sqrt(1 - dir.unit().y * dir.unit().y);

			int x = std::clamp(static_cast<int>(w * uv.x), 0, (int)w - 1);
			int y = std::clamp(static_cast<int>(h * uv.y), 0, (int)h - 1);
			float pdf = _pdf.at(y * w + x);

			return (abs(sin_theta) < EPS_F) ? 0.f : (h * w * pdf / (2 * sin_theta * PI_F * PI_F));
		}
	}

} // namespace Samplers
