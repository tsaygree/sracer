#include "geometry.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

class Light
{
public:
	/// @brief Default constructor
	Light() = delete;

	/// @brief Default constructor
	~Light() = default;

	/// @brief Constructor
	Light(const Vec3f& inPosition, const float& inIntencity)
		: position(inPosition)
		, intensity(inIntencity)
	{
	}

	/// @brief Position getter
	const Vec3f& getPosition() const
	{
		return position;
	}

	/// @brief Intensity getter
	float getIntensity() const
	{
		return intensity;
	}

protected:
	/// @brief Position of the light
	Vec3f position;

	/// @brief Intensity of the light
	float intensity;
};

class Material
{
public:
	/// @brief Constructor
	Material() = default;

	/// @brief Default destructor
	~Material() = default;

	/// @brief Parametrized constructor
	Material(float inRefractiveIndex, const Vec4f& inAlbedo, const Vec3f& inDiffuseColor, float inSpecularExponent)
		: refractiveIndex(inRefractiveIndex)
		, albedo(inAlbedo)
		, diffuseColor(inDiffuseColor)
		, specularExponent(inSpecularExponent)
	{
	}

	/// @brief refractiveIndex getter
	float getRefractiveIndex() const
	{
		return refractiveIndex;
	}

	/// @brief Albedo getter
	const Vec4f& getAlbedo() const
	{
		return albedo;
	}

	/// @brief diffusiveColor getter
	const Vec3f& getDiffuseColor() const
	{
		return diffuseColor;
	}

	/// @brief specularExponent getter
	float getSpecularExponent() const
	{
		return specularExponent;
	}

protected:
	/// @brief Refractive index of the material
	float refractiveIndex;

	/// @brief Albedo of the material
	/// @details Albedo is a vector of 4 values:
	/// 1. Diffuse reflection constant
	/// 2. Specular reflection constant
	/// 3. Reflection index
	/// 4. Refraction index
	Vec4f albedo;

	/// @brief Diffuse color of the material
	Vec3f diffuseColor;

	/// @brief Specular exponent of the material
	float specularExponent;
};

struct Sphere
{
	/// @brief Constructor
	Sphere(const Vec3f& inCenter, float inRadius, const Material& inMaterial)
		: center(inCenter)
		, radius(inRadius)
		, material(inMaterial)
	{
	}

	/// @brief Check if ray intersects with a sphere. Uses geometric solution.
	bool isIntersect(const Vec3f& rayOrigin, const Vec3f& rayDirection, float& hitDistance) const
	{
		const Vec3f originToSphereCenter = center - rayOrigin;
		const float originToSphereProjectionOnRay = originToSphereCenter * rayDirection;

		if (originToSphereProjectionOnRay < 0)
		{
			return false;
		}

		const float sphereCenterToRayDistanceSquared = originToSphereCenter * originToSphereCenter - originToSphereProjectionOnRay * originToSphereProjectionOnRay;

		if (sphereCenterToRayDistanceSquared > radius * radius)
		{
			return false;
		}

		const float hitPointDistanceDiff = sqrtf(radius * radius - sphereCenterToRayDistanceSquared);

		hitDistance = originToSphereProjectionOnRay - hitPointDistanceDiff;

		if (hitDistance < 0)
		{
			hitDistance = originToSphereProjectionOnRay + hitPointDistanceDiff;
		}

		return hitDistance >= 0;
	}

	/// @brief Center of the sphere in world space
	Vec3f center;

	/// @brief Radius of the sphere
	float radius;

	/// @brief Material of the sphere
	Material material;
};

/// @brief Calculates reflection vector
/// @param lightDirection light direction
/// @param normal surface normal
/// @return result reflection vector
Vec3f reflect(const Vec3f& lightDirection, const Vec3f& normal)
{
	return lightDirection - normal * 2.f * (lightDirection * normal);
}

/// @brief Calculates refraction vector
/// @param lightDirection light direction
/// @param normal surface normal
/// @param refractiveIndex refractive index of second medium
/// @return result refraction vector
Vec3f refract(const Vec3f& lightDirection, const Vec3f& normal, float refractiveIndex)
{
	Vec3f refractionNormal = normal;

	// treat first medium as air, thus refractive index is 1
	float firstMediumRefraciveIndex = 1.f;
	float secondMediumRefractiveIndex = refractiveIndex;

	float ligthToNormalCosine = lightDirection * normal;
	if (ligthToNormalCosine < 0)
	{
		// outside of the surface, take inverse of normal to get positive cosine
		ligthToNormalCosine = -ligthToNormalCosine;
	}
	else
	{
		// inside of the surface, invert normal and swap refractive indices
		refractionNormal = -normal;
		std::swap(firstMediumRefraciveIndex, secondMediumRefractiveIndex);
	}

	float refractiveIndexRatio = firstMediumRefraciveIndex / secondMediumRefractiveIndex;

	float reflectionIndex = 1.f - refractiveIndexRatio * refractiveIndexRatio * (1.f - ligthToNormalCosine * ligthToNormalCosine);

	if (reflectionIndex < 0)
	{
		// total internal reflection
		return Vec3f(0.f, 0.f, 0.f);
	}

	return lightDirection * refractiveIndexRatio +
		   refractionNormal * (refractiveIndexRatio * ligthToNormalCosine - sqrtf(reflectionIndex));
}

/// @brief Checks if ray intersects with any of the spheres in the scene
/// @param rayOrigin ray origin
/// @param rayDirection ray direction
/// @param spheres spheres in the scene
/// @param hitLocation result hit location
/// @param hitNormal result hit normal
/// @param material result material
bool isSceneIntersect(
	const Vec3f& rayOrigin, const Vec3f& rayDirection,
	const std::vector<Sphere>& spheres, Vec3f& hitLocation,
	Vec3f& hitNormal, Material& material)
{
	float sphereMinDistance = std::numeric_limits<float>::max();
	for (size_t sphereNumber = 0; sphereNumber < spheres.size(); ++sphereNumber)
	{
		const auto& sphere = spheres[sphereNumber];
		float hitDistance;
		if (sphere.isIntersect(rayOrigin, rayDirection, hitDistance) && hitDistance < sphereMinDistance)
		{
			sphereMinDistance = hitDistance;
			hitLocation = rayOrigin + rayDirection * hitDistance;
			hitNormal = (hitLocation - sphere.center).normalize();
			material = sphere.material;
		}
	}

	return sphereMinDistance < 1000;
}

/// @brief Cast ray into the scene and calculate result pixel color
/// @param rayOrigin ray origin
/// @param rayDirection ray direction
/// @param spheres spheres in the scene
/// @param lights lights in the scene
/// @param depth reflection depth
/// @return result pixel color
Vec3f castRay(const Vec3f& rayOrigin, const Vec3f& rayDirection,
	const std::vector<Sphere>& spheres, const std::vector<Light>& lights,
	size_t depth = 0)
{
	constexpr size_t maxDepth = 4;
	Vec3f hitLocation;
	Vec3f hitNormal;
	Material material;

	if (depth > maxDepth || isSceneIntersect(rayOrigin, rayDirection, spheres, hitLocation, hitNormal, material) == false)
	{
		return Vec3f(0.24f, 0.24f, 0.24f);
	}

	Vec3f reflectionDirection = reflect(rayDirection, hitNormal);
	Vec3f reflectionOrigin = reflectionDirection * hitNormal < 0 ? hitLocation - hitNormal * 1e-3 : hitLocation + hitNormal * 1e-3;
	Vec3f reflectionColor = castRay(reflectionOrigin, reflectionDirection, spheres, lights, depth + 1);

	Vec3f refractionDireciton = refract(rayDirection, hitNormal, material.getRefractiveIndex()).normalize();
	Vec3f refractionOrigin = refractionDireciton * hitNormal < 0 ? hitLocation - hitNormal * 1e-3 : hitLocation + hitNormal * 1e-3;
	Vec3f refractionColor = castRay(refractionOrigin, refractionDireciton, spheres, lights, depth + 1);

	float diffuseLightIntensity = 0;
	float specularLightIntensity = 0;
	for (size_t lightNumber = 0; lightNumber < lights.size(); ++lightNumber)
	{

		const auto& light = lights[lightNumber];
		Vec3f lightDirection = (light.getPosition() - hitLocation).normalize();
		float ligthDistance = (light.getPosition() - hitLocation).norm();

		Vec3f surfacePointOrigin =
			lightDirection * hitNormal < 0 ? hitLocation - hitNormal * 1e-3 : hitLocation + hitNormal * 1e-3;
		Vec3f lightObstacleHitLocation;
		Vec3f lightObstacleHitNormal;
		Material hitMaterial;
		const bool isSomethingLiesInLigthDirection = isSceneIntersect(surfacePointOrigin, lightDirection, spheres, lightObstacleHitLocation, lightObstacleHitNormal, hitMaterial);
		if (isSomethingLiesInLigthDirection)
		{
			const bool isHittedObjectLiesInFrontOfLight = (lightObstacleHitLocation - surfacePointOrigin).norm() < ligthDistance;
			if (isHittedObjectLiesInFrontOfLight)
			{
				continue;
			}
		}

		const float diffuseLigthStrength = lightDirection * hitNormal;

		diffuseLightIntensity += light.getIntensity() * std::max(0.f, diffuseLigthStrength);
		specularLightIntensity +=
			powf(std::max(0.f, reflect(lightDirection, hitNormal) * rayDirection), material.getSpecularExponent()) * light.getIntensity();
	}

	const auto& albedo = material.getAlbedo();

	return material.getDiffuseColor() * diffuseLightIntensity * albedo[0] +
		   Vec3f(1.f, 1.f, 1.f) * specularLightIntensity * albedo[1] +
		   reflectionColor * albedo[2] +
		   refractionColor * albedo[3];
}

/// @brief Outputs image to the file
/// @param spheres spheres in the scene
/// @param lights lights in the scene
void render(const std::vector<Sphere>& spheres, const std::vector<Light>& lights)
{
	constexpr int width = 1920;
	constexpr int height = 1080;
	constexpr float aspectRatio = width / (float)height;
	constexpr float screenDistance = 1.f;
	constexpr int fov = M_PI / 2.f;
	const float angleTangent = std::tan(fov / 2.f);

	std::vector<Vec3f> framebuffer(width * height);

	const Vec3f rayOrigin(0.f, 0.f, 0.f);
	for (size_t row = 0; row < height; ++row)
	{
		for (size_t col = 0; col < width; ++col)
		{
			// need to take aspect ratio into account for x coordinate
			const float rayDirectionX =
				(2 * (col + 0.5f) / float(width) - 1) * angleTangent * aspectRatio / screenDistance;

			// need to take negative value because y axis is inverted
			const float rayDirectionY =
				-(2 * (row + 0.5f) / float(height) - 1) * angleTangent / screenDistance;

			const Vec3f rayDirection = Vec3f(rayDirectionX, rayDirectionY, -1.f).normalize();

			framebuffer[col + row * width] = castRay(rayOrigin, rayDirection, spheres, lights);
		}
	}

	std::ofstream ofs;
	ofs.open("./sracerimage.ppm");

	ofs << "P6\n"
		<< width << " " << height << "\n255\n";

	for (size_t pixel = 0; pixel < height * width; ++pixel)
	{
		Vec3f& pixelColor = framebuffer[pixel];
		float max = std::max(pixelColor[0], std::max(pixelColor[1], pixelColor[2]));
		if (max > 1)
		{
			pixelColor = pixelColor * (1.f / max);
		}

		for (size_t colorNumber = 0; colorNumber < 3; ++colorNumber)
		{
			ofs << (char)(255 * std::max(0.f, std::min(1.f, framebuffer[pixel][colorNumber])));
		}
	}

	ofs.close();
}

int main()
{
	Material deepBlue(1.f, Vec4f(0.3f, 0.1f, 0.15f, 0.f), Vec3f(0.f, 0.f, 1.f), 50.f);
	Material ivory(1.f, Vec4f(0.6f, 0.3f, 0.2f, 0.f), Vec3f(0.4f, 0.4f, 0.3f), 50.f);
	Material redRubber(1.f, Vec4f(0.9f, 0.1f, 0.f, 0.f), Vec3f(0.3, 0.1, 0.1), 10.f);
	Material mirror(1.f, Vec4f(0.f, 10.f, 0.8f, 0.f), Vec3f(1.f, 1.f, 1.f), 1425.f);
	Material glass(100.f, Vec4f(0.f, 0.5f, 0.1f, 0.8f), Vec3f(0.6f, 0.7f, 0.8f), 125.f);

	std::vector<Sphere> spheres;
	// first row
	spheres.push_back(Sphere(Vec3f(-15.f, 10.f, -30.f), 4.f, mirror));
	spheres.push_back(Sphere(Vec3f(-5.f, 10.f, -30.f), 4.f, redRubber));
	spheres.push_back(Sphere(Vec3f(5.f, 10.f, -30.f), 4.f, redRubber));
	spheres.push_back(Sphere(Vec3f(15.f, 10.f, -30.f), 4.f, mirror));

	// second row
	spheres.push_back(Sphere(Vec3f(-15.f, 0.f, -30.f), 4.f, ivory));
	spheres.push_back(Sphere(Vec3f(-5.f, 0.f, -30.f), 4.f, glass));
	spheres.push_back(Sphere(Vec3f(5.f, 0.f, -30.f), 4.f, glass));
	spheres.push_back(Sphere(Vec3f(15.f, 0.f, -30.f), 4.f, ivory));

	// third row
	spheres.push_back(Sphere(Vec3f(-15.f, -10.f, -30.f), 4.f, mirror));
	spheres.push_back(Sphere(Vec3f(-5.f, -10.f, -30.f), 4.f, redRubber));
	spheres.push_back(Sphere(Vec3f(5.f, -10.f, -30.f), 4.f, redRubber));
	spheres.push_back(Sphere(Vec3f(15.f, -10.f, -30.f), 4.f, mirror));

	spheres.push_back(Sphere(Vec3f(0.f, 0.f, -60.f), 12.f, deepBlue));

	std::vector<Light> lights;
	lights.push_back(Light(Vec3f(-50.f, 50.f, 20.f), 2.f));
	lights.push_back(Light(Vec3f(22.f, -50.f, 0.f), 0.8f));
	lights.push_back(Light(Vec3f(-15.f, -15.f, -100.f), 0.8f));

	render(spheres, lights);

	return 0;
}