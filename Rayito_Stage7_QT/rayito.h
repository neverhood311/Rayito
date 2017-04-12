////////////////////////////////////////////////////////////////////////////////
//
// Very simple ray tracing example
//
////////////////////////////////////////////////////////////////////////////////

#ifndef __RAYITO_H__
#define __RAYITO_H__

#include "RMath.h"
#include "RRay.h"
#include "RMaterial.h"
#include "RScene.h"
#include "RLight.h"


namespace Rayito
{


//
// Image (collection of colored pixels with a width x height)
//

class Image
{
public:
    Image(size_t width, size_t height)
        : m_width(width), m_height(height), m_pixels(new Color[width * height]) { }
    
    virtual ~Image() { delete[] m_pixels; }
    
    size_t width()  const { return m_width; }
    size_t height() const { return m_height; }
    
    Color& pixel(size_t x, size_t y)
    {
        return m_pixels[y * m_width + x];
    }
    
protected:
    size_t m_width, m_height;
    Color *m_pixels;
};


//
// Cameras
//

class Camera
{
public:
    Camera(float shutterOpen = 0.0f, float shutterClose = 0.0f)
        : m_shutterOpen(shutterOpen), m_shutterClose(shutterClose) { }
    
    virtual ~Camera() { }
    
    // Generate a ray origin+direction for the camera, possibly with depth-of-field
    virtual Ray makeRay(float xScreen, float yScreen, float lensU, float lensV, float timeU) const = 0;
    
protected:
    float m_shutterOpen;
    float m_shutterClose;
    
    float time(float timeU) const { return m_shutterOpen + (m_shutterClose - m_shutterOpen) * timeU; }
};


class PerspectiveCamera : public Camera
{
public:
    // Create a perspective camera, with a given field-of-view in degrees,
    // look-at parameters, and depth-of-field parameters (lensRadius=0 to disable DOF)
    PerspectiveCamera(float fieldOfViewInDegrees,
                      const Point& origin,
                      const Vector& target,
                      const Vector& targetUpDirection,
                      float focalDistance,
                      float lensRadius,
                      float shutterOpen,
                      float shutterClose);
    
    virtual ~PerspectiveCamera() { }
    
    virtual Ray makeRay(float xScreen, float yScreen, float lensU, float lensV, float timeU) const;
    
protected:
    Point m_origin;
    Vector m_forward;
    Vector m_right;
    Vector m_up;
    float m_tanFov;
    float m_focalDistance;
    float m_lensRadius;
};


//
// Sampler container (for a given pixel, holds the samplers for all random features and bounces)
//

struct SamplerContainer
{
    // These are sampled once per pixel sample to start a path
    Sampler* m_lensSampler;
    Sampler* m_subpixelSampler;
    Sampler* m_timeSampler;
    // These are for picking a light source from which to start the light path
    Sampler* m_BDlightSelectionSampler; //pick a light
    Sampler* m_BDlightElementSampler;   //pick an element within the light
    Sampler* m_BDlightSampler;          //pick a point within the element
    Sampler* m_BDlightDirectionSampler; //pick a direction to fire a ray from the light

    // This is sampled once per bounce to determine the next leg of the path
    std::vector<Sampler*> m_bounceSamplers;
    // add a light bounce sampler for bidirectional path tracing (sampled once per light path bounce)
    std::vector<Sampler*> m_lightBounceSamplers;

    // These are sampled N times per bounce (once for each light sample)
    std::vector<Sampler*> m_lightSelectionSamplers; //pick a light
    std::vector<Sampler*> m_lightElementSamplers;   //pick an element within the light
    std::vector<Sampler*> m_lightSamplers;          //pick a point within the element
    std::vector<Sampler*> m_bsdfSamplers;
    
    unsigned int m_numLightSamples;
    unsigned int m_maxRayDepth; //the same for the eye path and the light path
    unsigned int m_minRayDepth; //0 means there's no minimum. Defaults to 3?
};

//
// Path vertex container (for a given pixel subsample, holds the vectors for path vertex information)
//

struct PathVertexContainer
{
    //lightpath geometric terms, edge_GeoTerm_L[max ray depth]
    std::vector<float> m_geoTerms_L;
    //eyepath geometric terms, edge_GeoTerm_E[max ray depth]
    std::vector<float> m_geoTerms_E;
    //lightpath specular flags, vert_isDirac_L[max ray depth], set all to false
    std::vector<bool> m_isDirac_L;
    //eyepath specular flags, vert_isDirac_E[max ray depth], set all to false
    std::vector<bool> m_isDirac_E;
    //lightpath PDFs with respect to Projected Solid Angle, vert_PDFPSA_L[max ray depth]
    std::vector<float> m_PdfPsa_L;
    //eyepath PDFs with respect to Projected Solid Angle, vert_PDFPSA_E[max ray depth]
    std::vector<float> m_PdfPsa_E;
    //lightpath vertex positions, vert_position_L[max ray depth]
    std::vector<Vector> m_position_L;
    //eyepath vertex positions, vert_position_E[max ray depth]
    std::vector<Vector> m_position_E;
    //lightpath vertex normals, vert_normal_L[max ray depth]
    std::vector<Vector> m_normal_L;
    //eyepath vertex normals, vert_normal_E[max ray depth]
    std::vector<Vector> m_normal_E;
    //lightpath vertex outgoing ray direction, vert_outdir_L[max ray depth]
    std::vector<Vector> m_outdir_L;
    //eyepath vertex outgoing ray direction, vert_outdir_E[max ray depth]
    std::vector<Vector> m_outdir_E;
    //lightpath vertex BSDFs, vert_BSDF_L[max ray depth]
    std::vector<Bsdf*> m_BSDF_L;
    //eyepath vertex BSDFs, vert_BSDF_E[max ray depth]
    std::vector<Bsdf*> m_BSDF_E;
    //lightpath vertex BSDF evaluation, vert_Fs_L[max ray depth]
    std::vector<Color> m_vert_Fs_L;
    //eyepath vertex BSDF evaluation, vert_Fs_E[max ray depth]
    std::vector<Color> m_vert_Fs_E;
    //vertex Alpha L sub i values, vert_Alpha_L_i[max ray depth]
    std::vector<float> m_alpha_i_L;
    //vertex Alpha E sub i values, vert_Alpha_E_i[max ray depth]
    std::vector<float> m_alpha_i_E;

    //combined path outgoing ray directions, xst_outdir[max ray depth * 2]
    std::vector<Vector> m_xst_outdir;
    //combined path Geometric terms, xst_GeoTerm[max ray depth * 2]
    std::vector<float> m_xst_GeoTerm;
};

//
// Ray tracing
//

// Path trace through the scene, starting with an initial ray.
// Pass along scene information and various samplers so that we can reduce noise
// along the way.
Color pathTrace(const Ray& ray,
                ShapeSet& scene,
                std::vector<Shape*>& lights,
                Rng& rng,
                SamplerContainer& samplers,
                unsigned int pixelSampleIndex);

// Do bidirectional path tracing through the scene
Color BDpathTrace(const Ray& ray,
                  ShapeSet& scene,
                  std::vector<Shape*>& lights,
                  Rng& rng,
                  SamplerContainer& samplers,
                  PathVertexContainer& path,
                  unsigned int pixelSampleIndex);

// Generate a ray-traced image of the scene, with the given camera, resolution,
// and sample settings
Image* raytrace(ShapeSet& scene,
                const Camera& cam,
                size_t width,
                size_t height,
                unsigned int pixelSamplesHint,
                unsigned int lightSamplesHint,
                unsigned int maxRayDepth);


} // namespace Rayito


#endif // __RAYITO_H__
