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

class IntersectionInfo
{
public:
	/// @brief Default constructor
	IntersectionInfo() = default;

	/// @brief Default destructor
	~IntersectionInfo() = default;

	/// @brief Constructor
	IntersectionInfo(bool inIntersected)
		: intersected(inIntersected)
	{
	}

	/// @brief Constructor
	IntersectionInfo(const Vec3f& inHitLocation, float inHitDistance, const Vec3f& inHitNormal, const Material& inHitMaterial, bool inIntersected)
		: hitLocation(inHitLocation)
		, hitDistance(inHitDistance)
		, hitNormal(inHitNormal)
		, intersected(inIntersected)
		, hitMaterial(inHitMaterial)
	{
	}

	/// @brief Hit location getter
	const Vec3f& getHitLocation() const
	{
		return hitLocation;
	}

	/// @brief Hit distance getter
	float getHitDistance() const
	{
		return hitDistance;
	}

	/// @brief Hit normal getter
	const Vec3f& getHitNormal() const
	{
		return hitNormal;
	}

	/// @brief Hit material getter
	const Material& getHitMaterial() const
	{
		return hitMaterial;
	}

	/// @brief Intersection flag getter
	bool isIntersected() const
	{
		return intersected;
	}

protected:
	/// @brief Intersection location
	Vec3f hitLocation;

	/// @brief Intersection distance
	float hitDistance;

	/// @brief Intersection normal
	Vec3f hitNormal;

	/// @brief Material of the hit object
	Material hitMaterial;

	/// @brief Intersection flag
	bool intersected;
};

class Shape
{
public:
	/// @brief Default constructor
	Shape() = default;

	/// @brief Default destructor
	virtual ~Shape() = default;

	/// @brief Copy constructor
	Shape(const Shape&) = delete;

	/// @brief Move constructor
	Shape(Shape&&) = delete;

	/// @brief Copy assignment
	Shape& operator=(const Shape&) = delete;

	/// @brief Move assignment
	Shape& operator=(Shape&&) = delete;

	/// @brief Checks if ray intersects with a shape
	/// @param rayOrigin origin of the ray
	/// @param rayDirection direction of the ray. Must be normalized
	/// @return intersection info
	virtual IntersectionInfo checkRayIntersection(const Vec3f& rayOrigin, const Vec3f& rayDirection) const = 0;

	/// @brief Material getter
	const Material& getMaterial() const
	{
		return material;
	}

protected:
	/// @brief Material of the shape
	Material material;
};

class Sphere : public Shape
{
public:
	/// @brief Default constructor
	Sphere() = delete;

	/// @brief Default destructor
	virtual ~Sphere() = default;

	/// @brief Constructor
	Sphere(const Vec3f& inCenter, float inRadius, const Material& inMaterial)
		: center(inCenter)
		, radius(inRadius)
	{
		material = inMaterial;
	}

	/// @brief Intersection check override. Uses geometric solution
	virtual IntersectionInfo checkRayIntersection(const Vec3f& rayOrigin, const Vec3f& rayDirection) const override
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

		float hitDistance = originToSphereProjectionOnRay - hitPointDistanceDiff;

		if (hitDistance < 0)
		{
			hitDistance = originToSphereProjectionOnRay + hitPointDistanceDiff;
		}

		const Vec3f hitLocation = rayOrigin + rayDirection * hitDistance;
		const Vec3f hitNormal = (hitLocation - center).normalize();

		return {hitLocation, hitDistance, hitNormal, material, hitDistance >= 0};
	}

	/// @brief Sphere center getter
	const Vec3f& getCenter() const
	{
		return center;
	}

	/// @brief Radius getter
	float getRadius() const
	{
		return radius;
	}

protected:
	/// @brief Center of the sphere in world space
	Vec3f center;

	/// @brief Radius of the sphere
	float radius;
};

class Plane : public Shape
{
public:
	/// @brief Default constructor
	Plane() = default;

	/// @brief Default destructor
	virtual ~Plane() = default;

	/// @brief Constructor
	Plane(const Vec3f& inNormalOrigin, const Vec3f& inNormal, const Material& inMaterial)
		: normalOrigin(inNormalOrigin)
		, normal(inNormal)
	{
		material = inMaterial;
	}

	/// @brief Intersection check override
	virtual IntersectionInfo checkRayIntersection(const Vec3f& rayOrigin, const Vec3f& rayDirection) const override
	{
		float planeNormalDotRayDirection = std::fabs(normal * rayDirection);
		if (planeNormalDotRayDirection <= 1e-3)
		{
			// ray is parallel to the plane => no intersection
			return false;
		}

		// hit distance can be calculated by combining plane equation and ray equation
		// plane equation: (planeHitLocation - normalOrigin) * normal = 0
		// ray equation (planeHitLocation equation) can be written as:
		// rayOrigin + rayDirection * hitDistance = planeHitLocation
		// thus:
		const float hitDistance = (normalOrigin - rayOrigin) * normal / (rayDirection * normal);

		const Vec3f hitLocation = rayOrigin + rayDirection * hitDistance;
		return {hitLocation, hitDistance, normal, material, hitDistance >= 0};
	}

	/// @brief Origin of the normal getter
	const Vec3f& getNormalOrigin() const
	{
		return normalOrigin;
	}

	/// @brief Normal getter
	const Vec3f& getNormal() const
	{
		return normal;
	}

protected:
	/// @brief Plane surface normal origin
	Vec3f normalOrigin;

	/// @brief Plane surface normal
	Vec3f normal;
};

class Rectangle : public Plane
{
public:
	/// @brief Default constructor
	Rectangle() = delete;

	/// @brief Default destructor
	virtual ~Rectangle() = default;

	/// @brief Constructor
	Rectangle(const Vec3f& inOrigin,
		const Vec3f& inFirstEdge, float inFirstEdgeLength,
		const Vec3f& inSecondEdge, float inSecondEdgeLength,
		const Material& inMaterial)
		: firstEdge(inFirstEdge)
		, firstEdgeLength(inFirstEdgeLength)
		, secondEdge(inSecondEdge)
		, secondEdgeLength(inSecondEdgeLength)
	{
		material = inMaterial;
		normal = cross(inFirstEdge, inSecondEdge).normalize();
		normalOrigin = inOrigin;
	}

	/// @brief Intersection check override
	virtual IntersectionInfo checkRayIntersection(const Vec3f& rayOrigin, const Vec3f& rayDirection) const override
	{
		const auto intersectionInfo = Plane::checkRayIntersection(rayOrigin, rayDirection);
		if (intersectionInfo.isIntersected() == false)
		{
			return intersectionInfo;
		}

		const Vec3f rectangleOriginToHit = intersectionInfo.getHitLocation() - normalOrigin;
		const float firstEdgeProjection = rectangleOriginToHit * firstEdge;
		const float secondEdgeProjection = rectangleOriginToHit * secondEdge;

		const bool isHitInsideRectangle = firstEdgeProjection >= 0 && firstEdgeProjection <= firstEdgeLength &&
										  secondEdgeProjection >= 0 && secondEdgeProjection <= secondEdgeLength;

		return {intersectionInfo.getHitLocation(), intersectionInfo.getHitDistance(), intersectionInfo.getHitNormal(), material, isHitInsideRectangle};
	}

protected:
	/// @brief Rectangle first edge direction
	Vec3f firstEdge;

	/// @brief Rectangle first edge length
	float firstEdgeLength;

	/// @brief Rectangle second edge direction
	Vec3f secondEdge;

	/// @brief Rectangle second edge length
	float secondEdgeLength;
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
/// @param shapes shapes in the scene
/// @return Intersection info
IntersectionInfo checkSceneIntersection(const Vec3f& rayOrigin, const Vec3f& rayDirection, const std::vector<Shape*>& shapes)
{
	float shapeMinHitDistance = std::numeric_limits<float>::max();
	Vec3f hitLocation;
	Vec3f hitNormal;
	Material hitMaterial;
	for (const auto shape : shapes)
	{
		const auto intersectionInfo = shape->checkRayIntersection(rayOrigin, rayDirection);
		const float hitDistance = intersectionInfo.getHitDistance();
		if (intersectionInfo.isIntersected() && hitDistance < shapeMinHitDistance)
		{
			shapeMinHitDistance = hitDistance;
			hitLocation = intersectionInfo.getHitLocation();
			hitNormal = intersectionInfo.getHitNormal();
			hitMaterial = intersectionInfo.getHitMaterial();
		}
	}

	return {hitLocation, shapeMinHitDistance, hitNormal, hitMaterial, shapeMinHitDistance < std::numeric_limits<float>::max()};
}

/// @brief Cast ray into the scene and calculate result pixel color
/// @param rayOrigin ray origin
/// @param rayDirection ray direction
/// @param shapes shapes in the scene
/// @param lights lights in the scene
/// @param depth reflection depth
/// @return result pixel color
Vec3f castRay(const Vec3f& rayOrigin, const Vec3f& rayDirection,
	const std::vector<Shape*>& shapes, const std::vector<Light*>& lights, size_t depth = 0)
{
	constexpr size_t maxDepth = 4;

	const auto intersectionInfo = checkSceneIntersection(rayOrigin, rayDirection, shapes);
	if (depth > maxDepth || intersectionInfo.isIntersected() == false)
	{
		return Vec3f(0.24f, 0.24f, 0.24f);
	}

	const auto& hitLocation = intersectionInfo.getHitLocation();
	const auto& hitNormal = intersectionInfo.getHitNormal();
	const auto& hitMaterial = intersectionInfo.getHitMaterial();

	const auto reflectionDirection = reflect(rayDirection, hitNormal);
	const auto reflectionOrigin = reflectionDirection * hitNormal < 0 ? hitLocation - hitNormal * 1e-3 : hitLocation + hitNormal * 1e-3;
	const auto reflectionColor = castRay(reflectionOrigin, reflectionDirection, shapes, lights, depth + 1);

	const auto refractionDireciton = refract(rayDirection, hitNormal, intersectionInfo.getHitMaterial().getRefractiveIndex()).normalize();
	const auto refractionOrigin = refractionDireciton * hitNormal < 0 ? hitLocation - hitNormal * 1e-3 : hitLocation + hitNormal * 1e-3;
	const auto refractionColor = castRay(refractionOrigin, refractionDireciton, shapes, lights, depth + 1);

	float diffuseLightIntensity = 0;
	float specularLightIntensity = 0;
	for (const auto light : lights)
	{
		Vec3f lightDirection = (light->getPosition() - hitLocation).normalize();
		float ligthDistance = (light->getPosition() - hitLocation).norm();

		Vec3f surfacePointOrigin =
			lightDirection * hitNormal < 0 ? hitLocation - hitNormal * 1e-3 : hitLocation + hitNormal * 1e-3;

		const auto obstacleIntersection = checkSceneIntersection(surfacePointOrigin, lightDirection, shapes);
		if (obstacleIntersection.isIntersected())
		{
			const auto& lightObstacleHitLocation = obstacleIntersection.getHitLocation();
			const bool isHittedObjectLiesInFrontOfLight = (lightObstacleHitLocation - surfacePointOrigin).norm() < ligthDistance;
			if (isHittedObjectLiesInFrontOfLight)
			{
				continue;
			}
		}

		const float diffuseLigthStrength = lightDirection * hitNormal;

		diffuseLightIntensity += light->getIntensity() * std::max(0.f, diffuseLigthStrength);
		specularLightIntensity +=
			powf(std::max(0.f, reflect(lightDirection, hitNormal) * rayDirection), hitMaterial.getSpecularExponent()) * light->getIntensity();
	}

	const auto& albedo = hitMaterial.getAlbedo();

	return hitMaterial.getDiffuseColor() * diffuseLightIntensity * albedo[0] +
		   Vec3f(1.f, 1.f, 1.f) * specularLightIntensity * albedo[1] +
		   reflectionColor * albedo[2] + refractionColor * albedo[3];
}

/// @brief Outputs image to the file
/// @param shapes shapes in the scene
/// @param lights lights in the scene
void render(const std::vector<Shape*>& shapes, const std::vector<Light*>& lights)
{
	constexpr int width = 1920;
	constexpr int height = 1080;
	constexpr float aspectRatio = width / (float)height;
	constexpr float screenDistance = 1.f;
	constexpr float fov = 1.05;
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

			framebuffer[col + row * width] = castRay(rayOrigin, rayDirection, shapes, lights);
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

	std::vector<Shape*> shapes{
		new Sphere(Vec3f(-15.f, 10.f, -30.f), 4.f, mirror),
		new Sphere(Vec3f(-5.f, 10.f, -30.f), 4.f, redRubber),
		new Sphere(Vec3f(5.f, 10.f, -30.f), 4.f, redRubber),
		new Sphere(Vec3f(15.f, 10.f, -30.f), 4.f, mirror),
		new Sphere(Vec3f(-15.f, 0.f, -30.f), 4.f, ivory),
		new Sphere(Vec3f(-5.f, 0.f, -30.f), 4.f, glass),
		new Sphere(Vec3f(5.f, 0.f, -30.f), 4.f, glass),
		new Sphere(Vec3f(15.f, 0.f, -30.f), 4.f, ivory),
		new Sphere(Vec3f(-15.f, -10.f, -30.f), 4.f, mirror),
		new Sphere(Vec3f(-5.f, -10.f, -30.f), 4.f, redRubber),
		new Sphere(Vec3f(5.f, -10.f, -30.f), 4.f, redRubber),
		new Sphere(Vec3f(15.f, -10.f, -30.f), 4.f, mirror),
		new Sphere(Vec3f(0.f, 0.f, -60.f), 12.f, deepBlue),
		new Rectangle(Vec3f(0.f, -25.f, -70.f),
			Vec3f(1.f, 0.f, 0.65f).normalize(), 60.f,
			Vec3f(0.f, 1.f, 0.f).normalize(), 50.f,
			mirror)};

	std::vector<Light*> lights{
		new Light(Vec3f(-50.f, 50.f, 20.f), 2.f),
		new Light(Vec3f(22.f, -50.f, 0.f), 0.8f),
		new Light(Vec3f(-15.f, -15.f, -100.f), 0.8f)};

	render(shapes, lights);

	for (const auto shape : shapes)
	{
		delete shape;
	}
	shapes.clear();

	for (const auto light : lights)
	{
		delete light;
	}
	lights.clear();

	return 0;
}