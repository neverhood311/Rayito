#include <string>
#include <iostream>
#include <sstream>

#include "rayito.h"

#include <QThread>


using namespace Rayito;


// A few debug print helpers, in case we need them.

std::ostream& operator <<(std::ostream& stream, const Color& c)
{
    stream << '(' << c.m_r << ", " << c.m_g << ", " << c.m_b << ')';
    return stream;
}

std::ostream& operator <<(std::ostream& stream, const Vector& v)
{
    stream << '[' << v.m_x << ", " << v.m_y << ", " << v.m_z << ']';
    return stream;
}

std::ostream& operator <<(std::ostream& stream, const Quaternion& q)
{
    stream << '[' << q.m_w << "; " << q.m_v.m_x << ", " << q.m_v.m_y << ", " << q.m_v.m_z << ']';
    return stream;
}

std::ostream& operator <<(std::ostream& stream, const Ray& r)
{
    stream << r.m_origin << ":" << r.m_direction << ":[" << kRayTMin << ' ' << r.m_tMax << "]@" << r.m_time;
    return stream;
}


namespace
{


//
// RenderThread works on a small chunk of the image
//
class RenderThread : public QThread
{
public:
    RenderThread(size_t xstart, size_t xend, size_t ystart, size_t yend,
                 Image *pImage,
                 ShapeSet& masterSet,
                 const Camera& cam,
                 std::vector<Shape*>& lights,
                 unsigned int pixelSamplesHint,
                 unsigned int lightSamplesHint,
                 unsigned int maxRayDepth)
        : m_xstart(xstart), m_xend(xend), m_ystart(ystart), m_yend(yend),
          m_pImage(pImage), m_masterSet(masterSet), m_camera(cam), m_lights(lights),
          m_pixelSamplesHint(pixelSamplesHint), m_lightSamplesHint(lightSamplesHint),
          m_maxRayDepth(maxRayDepth) { }
    
protected:
    virtual void run()
    {
        // Random number generator (for random pixel positions, light positions, etc)
        // We seed the generator for this render thread based on something that
        // doesn't change, but gives us a good variable seed for each thread.
        Rng rng(static_cast<unsigned int>(((m_xstart << 16) | m_xend) ^ m_xstart),
                static_cast<unsigned int>(((m_ystart << 16) | m_yend) ^ m_ystart));
        
        // The aspect ratio is used to make the image only get more zoomed in when
        // the height changes (and not the width)
        float aspectRatioXToY = float(m_pImage->width()) / float(m_pImage->height());
        
        SamplerContainer samplers;
        samplers.m_numLightSamples = m_lights.empty() ? 0 : m_lightSamplesHint * m_lightSamplesHint;
        samplers.m_maxRayDepth = m_maxRayDepth;
        samplers.m_minRayDepth = 3; //TODO: this should be set by the GUI
        
        // Set up samplers for each of the ray bounces.  Each bounce will use
        // the same sampler for all pixel samples in the pixel to reduce noise.
        for (size_t i = 0; i < m_maxRayDepth; ++i)
        {
            samplers.m_bounceSamplers.push_back(new CorrelatedMultiJitterSampler(m_pixelSamplesHint,
                                                                                 m_pixelSamplesHint,
                                                                                 rng,
                                                                                 rng.nextUInt32()));
            samplers.m_lightBounceSamplers.push_back(new CorrelatedMultiJitterSampler(m_pixelSamplesHint,
                                                                                      m_pixelSamplesHint,
                                                                                      rng,
                                                                                      rng.nextUInt32()));
        }
        for(size_t i = 0; i < m_maxRayDepth * 2; i++){
            samplers.m_lightSelectionSamplers.push_back(new CorrelatedMultiJitterSampler(m_pixelSamplesHint * m_lightSamplesHint *
                                                                                         m_pixelSamplesHint * m_lightSamplesHint,
                                                                                         rng,
                                                                                         rng.nextUInt32()));
            samplers.m_lightElementSamplers.push_back(new CorrelatedMultiJitterSampler(m_pixelSamplesHint * m_lightSamplesHint *
                                                                                       m_pixelSamplesHint * m_lightSamplesHint,
                                                                                       rng,
                                                                                       rng.nextUInt32()));
            samplers.m_lightSamplers.push_back(new CorrelatedMultiJitterSampler(m_pixelSamplesHint * m_lightSamplesHint,
                                                                                m_pixelSamplesHint * m_lightSamplesHint,
                                                                                rng,
                                                                                rng.nextUInt32()));
            samplers.m_bsdfSamplers.push_back(new CorrelatedMultiJitterSampler(m_pixelSamplesHint * m_lightSamplesHint,
                                                                               m_pixelSamplesHint * m_lightSamplesHint,
                                                                               rng,
                                                                               rng.nextUInt32()));
        }
        // Set up samplers for russian roulette. We only need maxBounces - minBounces of them
        for(size_t i = 0; i < m_maxRayDepth - samplers.m_minRayDepth; i++){
            samplers.m_eyepathRussianRouletteSamplers.push_back(new CorrelatedMultiJitterSampler(m_pixelSamplesHint * m_pixelSamplesHint,
                                                                                                rng,
                                                                                                rng.nextUInt32()));
            samplers.m_lightpathRussianRouletteSamplers.push_back(new CorrelatedMultiJitterSampler(m_pixelSamplesHint * m_pixelSamplesHint,
                                                                                                rng,
                                                                                                rng.nextUInt32()));
        }

        // Set up samplers for each pixel sample
        samplers.m_timeSampler = new CorrelatedMultiJitterSampler(m_pixelSamplesHint * m_pixelSamplesHint, rng, rng.nextUInt32());
        samplers.m_lensSampler = new CorrelatedMultiJitterSampler(m_pixelSamplesHint, m_pixelSamplesHint, rng, rng.nextUInt32());
        samplers.m_subpixelSampler = new CorrelatedMultiJitterSampler(m_pixelSamplesHint, m_pixelSamplesHint, rng, rng.nextUInt32());
        samplers.m_BDlightSelectionSampler = new CorrelatedMultiJitterSampler(m_pixelSamplesHint * m_pixelSamplesHint, rng, rng.nextUInt32());
        samplers.m_BDlightElementSampler = new CorrelatedMultiJitterSampler(m_pixelSamplesHint * m_pixelSamplesHint, rng, rng.nextUInt32());
        samplers.m_BDlightSampler = new CorrelatedMultiJitterSampler(m_pixelSamplesHint, m_pixelSamplesHint, rng, rng.nextUInt32());
        samplers.m_BDlightDirectionSampler = new CorrelatedMultiJitterSampler(m_pixelSamplesHint, m_pixelSamplesHint, rng, rng.nextUInt32());
        unsigned int totalPixelSamples = samplers.m_subpixelSampler->total2DSamplesAvailable();

        // Set up the PathVertexContainer with the right number of entries
        PathVertexContainer path;
        //for (max ray depth)
        for(size_t i = 0; i < m_maxRayDepth+1; i++){
            //add false to isDirac_L and isDirac_E
            path.m_vert_isDirac_L.push_back(false);
            path.m_vert_isDirac_E.push_back(false);
            //add zeros to PdfPsa_L and PdfPsa_E
            //path.m_PdfPsa_L.push_back(0.0f);
            //path.m_PdfPsa_E.push_back(0.0f);
            //add (0,0,0)s to position_L and position_E
            path.m_vert_position_L.push_back(new Vector(0.0f));
            path.m_vert_position_E.push_back(new Vector(0.0f));
            //add (0,0,0)s to normal_L and normal_E
            path.m_vert_normal_L.push_back(new Vector(0.0f));
            path.m_vert_normal_E.push_back(new Vector(0.0f));
            //add (0,0,0)s to outdir_L and outdir_E
            //path.m_outdir_L.push_back(new Vector(0.0f));
            //path.m_outdir_E.push_back(new Vector(0.0f));
            //add NULLs to BSDF_L and BSDF_E
            path.m_vert_BSDF_L.push_back(NULL);
            path.m_vert_BSDF_E.push_back(NULL);
            path.m_vert_pShape_L.push_back(NULL);
            path.m_vert_pShape_E.push_back(NULL);
            path.m_vert_pMaterial_L.push_back(NULL);
            path.m_vert_pMaterial_E.push_back(NULL);
            //add (1,1,1)s to matColor_L and matColor_E
            path.m_vert_matColor_L.push_back(new Color(1.0f, 1.0f, 1.0f));
            path.m_vert_matColor_E.push_back(new Color(1.0f, 1.0f, 1.0f));
            path.m_vert_colorModifier_L.push_back(new Color(1.0f, 1.0f, 1.0f));
            path.m_vert_colorModifier_E.push_back(new Color(1.0f, 1.0f, 1.0f));
            //add (1,1,1)s to vert_Fs_L and vert_Fs_E
            //path.m_vert_Fs_L.push_back(new Color(1.0f, 1.0f, 1.0f));
            //path.m_vert_Fs_E.push_back(new Color(1.0f, 1.0f, 1.0f));
            //add zeros to vert_PA_L and vert_PA_E
            //path.m_vert_PA_L.push_back(0.0f);
            //path.m_vert_PA_E.push_back(0.0f);
            path.m_vert_BSDF_weight_L.push_back(0.0f);
            path.m_vert_BSDF_weight_E.push_back(0.0f);
            path.m_PdfSa_L.push_back(0.0f);
            path.m_PdfSa_E.push_back(0.0f);
            path.m_vert_BSDF_result_L.push_back(1.0f);
            path.m_vert_BSDF_result_E.push_back(1.0f);
            path.m_vert_outdir_L.push_back(new Vector(0.0f));
            path.m_vert_outdir_E.push_back(new Vector(0.0f));
        }

        // For each pixel row...
        for (size_t y = m_ystart; y < m_yend; ++y)
        {
            // For each pixel across the row...
            for (size_t x = m_xstart; x < m_xend; ++x)
            {
                /*if(x == 224 && y == 348){
                    //perfect reflection towards background
                    int one = 0;
                    one++;
                }
                if(x == 229 && y == 329){
                    //perfect reflection towards light source
                    int one = 0;
                    one++;
                }
                if(x == 220 && y == 365){
                    //perfect reflection towards ground plane
                    int one = 0;
                    one++;
                }
                //310 20
                if(x == 310 && y == 20){
                    //hit the light source first
                    int one = 0;
                    one++;
                }*/
                // Accumulate pixel color
                Color pixelColor(0.0f, 0.0f, 0.0f);
                // For each sample in the pixel...
                for (size_t psi = 0; psi < totalPixelSamples; ++psi)
                {
                    // Calculate a stratified random position within the pixel
                    // to hide aliasing
                    float pu, pv;
                    samplers.m_subpixelSampler->sample2D(psi, pu, pv);
                    float xu = (x + pu) / float(m_pImage->width());
                    // Flip pixel row to be in screen space (images are top-down)
                    float yu = 1.0f - (y + pv) / float(m_pImage->height());
                    
                    // Calculate a stratified random variation for depth-of-field
                    float lensU, lensV;
                    samplers.m_lensSampler->sample2D(psi, lensU, lensV);
                    
                    // Grab a time for motion blur
                    float timeU = samplers.m_timeSampler->sample1D(psi);
                    
                    // Find where this pixel sample hits in the scene
                    Ray ray = m_camera.makeRay((xu - 0.5f) * aspectRatioXToY + 0.5f,
                                               yu,
                                               lensU,
                                               lensV,
                                               timeU);
                    
                    // Trace a path out, gathering estimated radiance along the path
                    pixelColor += BDpathTrace(ray,
                                            m_masterSet,
                                            m_lights,
                                            rng,
                                            samplers,
                                            path,
                                            psi);
                }
                // Divide by the number of pixel samples (a box pixel filter, essentially)
                pixelColor /= totalPixelSamples;
                
                // Store off the computed pixel in a big buffer
                m_pImage->pixel(x, y) = pixelColor;
                
                // Reset samplers for the next pixel sample
                for (size_t i = 0; i < m_maxRayDepth; ++i)
                {
                    samplers.m_bounceSamplers[i]->refill(rng.nextUInt32());
                    samplers.m_lightBounceSamplers[i]->refill(rng.nextUInt32());
                }
                for(size_t i = 0; i < m_maxRayDepth * 2; ++i){
                    samplers.m_lightSelectionSamplers[i]->refill(rng.nextUInt32());
                    samplers.m_lightElementSamplers[i]->refill(rng.nextUInt32());
                    samplers.m_lightSamplers[i]->refill(rng.nextUInt32());
                    samplers.m_bsdfSamplers[i]->refill(rng.nextUInt32());
                }
                for(size_t i = 0; i < m_maxRayDepth - samplers.m_minRayDepth; i++){
                    samplers.m_eyepathRussianRouletteSamplers[i]->refill(rng.nextUInt32());
                    samplers.m_lightpathRussianRouletteSamplers[i]->refill(rng.nextUInt32());
                }
                samplers.m_lensSampler->refill(rng.nextUInt32());
                samplers.m_timeSampler->refill(rng.nextUInt32());
                samplers.m_subpixelSampler->refill(rng.nextUInt32());
                samplers.m_BDlightSelectionSampler->refill(rng.nextUInt32());
                samplers.m_BDlightElementSampler->refill(rng.nextUInt32());
                samplers.m_BDlightSampler->refill(rng.nextUInt32());
                samplers.m_BDlightDirectionSampler->refill(rng.nextUInt32());

                // Reset the PathVertexContainer for the next pixel sample?
                //TODO
                //set the isDirac vectors to false?
            }
        }
        
        // Deallocate all samplers
        for (size_t i = 0; i < m_maxRayDepth; ++i)
        {
            delete samplers.m_bounceSamplers[i];
            delete samplers.m_lightBounceSamplers[i];
        }
        for(size_t i = 0; i < m_maxRayDepth * 2; ++i){
            delete samplers.m_lightSelectionSamplers[i];
            delete samplers.m_lightElementSamplers[i];
            delete samplers.m_lightSamplers[i];
            delete samplers.m_bsdfSamplers[i];
        }
        for(size_t i = 0; i < m_maxRayDepth - samplers.m_minRayDepth; i++){
            delete samplers.m_eyepathRussianRouletteSamplers[i];
            delete samplers.m_lightpathRussianRouletteSamplers[i];
        }
        delete samplers.m_lensSampler;
        delete samplers.m_timeSampler;
        delete samplers.m_subpixelSampler;
        delete samplers.m_BDlightSelectionSampler;
        delete samplers.m_BDlightElementSampler;
        delete samplers.m_BDlightSampler;
        delete samplers.m_BDlightDirectionSampler;

        // Deallocate path Vectors and Colors
        for(size_t i = 0; i < m_maxRayDepth+1; i++){
            //delete path.m_outdir_L[i];
            //delete path.m_outdir_E[i];
            //delete path.m_vert_Fs_L[i];
            //delete path.m_vert_Fs_E[i];
            delete path.m_vert_position_L[i];
            delete path.m_vert_position_E[i];
            delete path.m_vert_normal_L[i];
            delete path.m_vert_normal_E[i];
            delete path.m_vert_matColor_L[i];
            delete path.m_vert_matColor_E[i];
            delete path.m_vert_colorModifier_L[i];
            delete path.m_vert_colorModifier_E[i];
            delete path.m_vert_outdir_L[i];
            delete path.m_vert_outdir_E[i];
        }
    }
    
    size_t m_xstart, m_xend, m_ystart, m_yend;
    Image *m_pImage;
    ShapeSet& m_masterSet;
    const Camera& m_camera;
    std::vector<Shape*>& m_lights;
    unsigned int m_pixelSamplesHint, m_lightSamplesHint;
    unsigned int m_maxRayDepth;
};


} // namespace


namespace Rayito
{


// Construct a perspective camera, precomputing a few things to ray trace faster
PerspectiveCamera::PerspectiveCamera(float fieldOfViewInDegrees,
                                     const Point& origin,
                                     const Vector& target,
                                     const Vector& targetUpDirection,
                                     float focalDistance,
                                     float lensRadius,
                                     float shutterOpen,
                                     float shutterClose)
    : Camera(shutterOpen, shutterClose),
      m_origin(origin),
      m_forward((target - origin).normalized()),
      m_tanFov(std::tan(fieldOfViewInDegrees * M_PI / 180.0f)),
      m_focalDistance(focalDistance),
      m_lensRadius(lensRadius)
{
    m_right = cross(m_forward, targetUpDirection);
    m_up = cross(m_right, m_forward); // no need to normalize, it already is
}

Ray PerspectiveCamera::makeRay(float xScreen, float yScreen, float lensU, float lensV, float timeU) const
{
    // Set up a camera ray given the look-at spec, FOV, and screen position to aim at
    
    // Set up ray info
    Ray ray;
    ray.m_origin = m_origin;
    ray.m_direction = m_forward +
                      m_right * ((xScreen - 0.5f) * m_tanFov) +
                      m_up * ((yScreen - 0.5f) * m_tanFov);
    ray.m_direction.normalize();
    ray.m_time = time(timeU);
    
    if (m_lensRadius > 0)
    {
        // Modify it for DOF if necessary
        float horizontalShift = 0.0f, verticalShift = 0.0f;
        float nearDistance = 0.0f; // TODO: be able to set a custom near distance?

        // Compute shifts on the near plane due to DOF
        uniformToUniformDisk(lensU, lensV, horizontalShift, verticalShift);
        horizontalShift *= m_lensRadius;
        verticalShift *= m_lensRadius;

        // Compute local direction rays for computing focal parameters
        Vector localRayDirection((xScreen - 0.5f) * m_tanFov,
                                 (yScreen - 0.5f) * m_tanFov,
                                 1.0f);
        localRayDirection.normalize();

        // Compute primary ray focal plane intersection
        float focusT = (m_focalDistance - nearDistance) / localRayDirection.m_z;
        Point focusPoint = m_origin + ray.m_direction * focusT;

        // Modify primary ray origin
        ray.m_origin += m_right * horizontalShift + m_up * verticalShift;

        // Compute primary ray direction
        ray.m_direction = focusPoint - ray.m_origin;
        ray.m_direction.normalize();
    }
    
    return ray;
}


Color pathTrace(const Ray& ray,
                ShapeSet& scene,
                std::vector<Shape*>& lights,
                Rng& rng,
                SamplerContainer& samplers,
                unsigned int pixelSampleIndex)
{
    // Accumulate total incoming radiance in 'result'
    Color result = Color(0.0f, 0.0f, 0.0f);
    // As we get through more and more bounces, we track how much the light is
    // diminished through each bounce
    Color throughput = Color(1.0f, 1.0f, 1.0f);
    
    // Start with the initial ray from the camera
    Ray currentRay = ray;
    
    // While we have bounces left we can still take...
    size_t numBounces = 0;
    size_t numDiracBounces = 0;
    bool lastBounceDiracDistribution = false;
    while (numBounces < samplers.m_maxRayDepth)
    {
        // Trace the ray to see if we hit anything
        Intersection intersection(currentRay);
        if (!scene.intersect(intersection))
        {
            // No hit, return black (background)
            break;
        }
        
        // Add in emission when directly visible from the camera or if the
        // last bounce was pure specular, as this is the only way to account
        // for the light. Also if the encountered surface is *not* a light, we
        // should always add its emission.
        if (!intersection.m_pShape->isLight() || numBounces == 0 || lastBounceDiracDistribution)
        {
            result += throughput * intersection.m_pMaterial->emittance();
        }
        
        // Evaluate the material and intersection information at this bounce
        Point position = intersection.position();
        Vector normal = intersection.m_normal;
        Vector outgoing = -currentRay.m_direction;
        Bsdf* pBsdf = NULL;
        float bsdfWeight = 1.0f;
        Color matColor = intersection.m_pMaterial->evaluate(position,
                                                            normal,
                                                            outgoing,
                                                            pBsdf,
                                                            bsdfWeight);
        // No BSDF?  We can't evaluate lighting, so bail.
        if (pBsdf == NULL)
        {
            return result;
        }
        
        // Was this a perfect specular bounce?
        lastBounceDiracDistribution = pBsdf->isDiracDistribution();
        if (lastBounceDiracDistribution)
            numDiracBounces++;
        
        // Evaluate direct lighting at this bounce
        
        if (!lastBounceDiracDistribution)
        {
            Color lightResult = Color(0.0f, 0.0f, 0.0f);
            float lightSelectionWeight = float(lights.size()) / samplers.m_numLightSamples;
            for (size_t lightSampleIndex = 0; lightSampleIndex < samplers.m_numLightSamples; ++lightSampleIndex)
            {
                // Sample lights using MIS between the light and the BSDF.
                // This means we ask the light for a direction, and the likelihood
                // of having sampled that direction (the PDF).  Then we ask the
                // BSDF what it thinks of that direction (its PDF), and weight
                // the light sample with MIS.
                //
                // Then, we ask the BSDF for a direction, and the likelihood of
                // having sampled that direction (the PDF).  Then we ask the
                // light what it thinks of that direction (its PDF, and whether
                // that direction even runs into the light at all), and weight
                // the BSDF sample with MIS.
                //
                // By doing both samples and asking both the BSDF and light for
                // their PDF for each one, we can combine the strengths of both
                // sampling methods and get the best of both worlds.  It does
                // cost an extra shadow ray and evaluation, though, but it is
                // generally such an improvement in quality that it is very much
                // worth the overhead.
                
                // Select a light randomly for this sample
                unsigned int finalLightSampleIndex = pixelSampleIndex * samplers.m_numLightSamples +
                                                     lightSampleIndex;
                float liu = samplers.m_lightSelectionSamplers[numBounces]->sample1D(finalLightSampleIndex);
                size_t lightIndex = (size_t)(liu * lights.size());
                if (lightIndex >= lights.size())
                    lightIndex = lights.size() - 1;
                Light *pLightShape = (Light*) lights[lightIndex];
                
                // Ask the light for a random position/normal we can use for lighting
                float lsu, lsv;
                samplers.m_lightSamplers[numBounces]->sample2D(finalLightSampleIndex, lsu, lsv);
                float leu = samplers.m_lightElementSamplers[numBounces]->sample1D(finalLightSampleIndex);
                Point lightPoint;
                Vector lightNormal;
                float lightPdf = 0.0f;
                pLightShape->sampleSurface(position,
                                           normal,
                                           ray.m_time,
                                           lsu, lsv, leu,
                                           lightPoint,
                                           lightNormal,
                                           lightPdf);
                
                if (lightPdf > 0.0f)
                {   
                    // Ask the BSDF what it thinks of this light position (for MIS)
                    Vector lightIncoming = position - lightPoint;
                    float lightDistance = lightIncoming.normalize();
                    float bsdfPdf = 0.0f;
                    float bsdfResult = pBsdf->evaluateSA(lightIncoming,
                                                         outgoing,
                                                         normal,
                                                         bsdfPdf);
                    if (bsdfResult > 0.0f && bsdfPdf > 0.0f)
                    {
                        // Fire a shadow ray to make sure we can actually see the light position
                        Ray shadowRay(position, -lightIncoming, lightDistance - kRayTMin, ray.m_time);
                        if (!scene.doesIntersect(shadowRay))
                        {
                            // The light point is visible, so let's add that
                            // contribution (mixed by MIS)
                            float misWeightLight = powerHeuristic(1, lightPdf, 1, bsdfPdf);
                            lightResult += pLightShape->emitted() *
                                           intersection.m_colorModifier * matColor *
                                           bsdfResult *
                                           std::fabs(dot(-lightIncoming, normal)) *
                                           misWeightLight / (lightPdf * bsdfWeight);
                        }
                    }
                }
                
                // Ask the BSDF for a sample direction
                float bsu, bsv;
                samplers.m_bsdfSamplers[numBounces]->sample2D(finalLightSampleIndex, bsu, bsv);
                Vector bsdfIncoming;
                float bsdfPdf = 0.0f;
                float bsdfResult = pBsdf->sampleSA(bsdfIncoming,
                                                   outgoing,
                                                   normal,
                                                   bsu,
                                                   bsv,
                                                   bsdfPdf);
                if (bsdfPdf > 0.0f && bsdfResult > 0.0f)
                {
                    Intersection shadowIntersection(Ray(position, -bsdfIncoming, kRayTMax, ray.m_time));
                    bool intersected = scene.intersect(shadowIntersection);
                    if (intersected && shadowIntersection.m_pShape == pLightShape)
                    {
                        // Ask the light what it thinks of this direction (for MIS)
                        lightPdf = pLightShape->intersectPdf(shadowIntersection);
                        if (lightPdf > 0.0f)
                        {
                            // BSDF chose the light, so let's add that
                            // contribution (mixed by MIS)
                            float misWeightBsdf = powerHeuristic(1, bsdfPdf, 1, lightPdf);
                            lightResult += pLightShape->emitted() * 
                                           intersection.m_colorModifier * matColor * bsdfResult *
                                           std::fabs(dot(-bsdfIncoming, normal)) * misWeightBsdf /
                                           (bsdfPdf * bsdfWeight);
                        }
                    }
                }
            }
            
            // Average light samples
            lightResult *= samplers.m_numLightSamples > 0 ? lightSelectionWeight : 0.0f;
            
            // Add direct lighting at this bounce (modified by how much the
            // previous bounces have dimmed it)
            result += throughput * lightResult;
        }
                
        // Sample the BSDF to find the direction the next leg of the path goes in
        float bsdfSampleU, bsdfSampleV;
        samplers.m_bounceSamplers[numBounces]->sample2D(pixelSampleIndex, bsdfSampleU, bsdfSampleV);
        Vector incoming;
        float incomingBsdfPdf = 0.0f;
        float incomingBsdfResult = pBsdf->sampleSA(incoming,
                                                   outgoing,
                                                   normal,
                                                   bsdfSampleU,
                                                   bsdfSampleV,
                                                   incomingBsdfPdf);

        if (incomingBsdfPdf > 0.0f)
        {
            currentRay.m_origin = position;
            currentRay.m_direction = -incoming;
            currentRay.m_tMax = kRayTMax;
            // Reduce lighting effect for the next bounce based on this bounce's BSDF
            throughput *= intersection.m_colorModifier * matColor * incomingBsdfResult *
                          (std::fabs(dot(-incoming, normal)) /
                          (incomingBsdfPdf * bsdfWeight));
        }
        else
        {
            break; // BSDF is zero, stop bouncing
        }
        
        numBounces++;
    }
    
    // This represents an estimate of the total light coming in along the path
    return result;
}

/*
//bidirectional pathTrace function
Color BDpathTrace_z(const Ray& ray,
                  ShapeSet& scene,
                  std::vector<Shape*>& lights,
                  Rng& rng,
                  SamplerContainer& samplers,
                  PathVertexContainer& path,
                  unsigned int pixelSampleIndex){
    //initialize the final result
    Color result = Color(0.0f, 0.0f, 0.0f);

    //BUILD A PATH STARTING FROM THE EYE
    //initialize the eye throughput (for russian roulette) to (1,1,1)
    Color eyeThroughput = Color(1.0f, 1.0f, 1.0f);

    //set the first vert_position to the camera's position
    path.m_vert_position_E[0]->set(ray.m_origin);
    //set the first raydir
    path.m_outdir_E[0]->set(-ray.m_direction);

    //initialize the eye vertex count, nE, to 1
    size_t nE = 1;
    //start with the initial ray from the camera
    Ray currentRay = ray;
    size_t numBounces = 0;
    //while depth < max depth
    while(numBounces < samplers.m_maxRayDepth){
        //if numbounces >= min depth, do russian roulette
        if(false && numBounces >= samplers.m_minRayDepth){
            //get the maximum component of the eye throughput
            float maxComp = std::max(std::max(eyeThroughput.m_r, eyeThroughput.m_g), eyeThroughput.m_b);
            //get a random number
            float rrSample = samplers.m_eyepathRussianRouletteSamplers[numBounces - samplers.m_minRayDepth]->sample1D(pixelSampleIndex);
            //if the random number is greater than the max component
            if(rrSample > maxComp){
                //you've been killed
                break;
            }
        }
        //Trace the ray to see if we hit anything
        Intersection intersection(currentRay);
        //if we didn't
        if(!scene.intersect(intersection)){
            //end the eyepath here
            break;
        }
        // Evaluate the material and intersection information at this bounce
        //set vertex position[nE]
        Point position = *path.m_vert_position_E[nE] = intersection.position();
        //set vertex normal[nE]
        Vector normal = *path.m_vert_normal_E[nE] = intersection.m_normal;
        //set outdir_E[nE-1] (remember to negate it here)
        Vector outgoing = *path.m_outdir_E[nE-1] = -currentRay.m_direction;
        Bsdf* pBsdf = NULL;
        float bsdfWeight = 1.0f;
        Color matColor = *path.m_vert_matColor_E[nE] = intersection.m_pMaterial->evaluate(position,
                                                            normal,
                                                            outgoing,
                                                            pBsdf,
                                                            bsdfWeight);
        // No BSDF? We can't evaluate lighting, so bail.
        if(pBsdf == NULL){
            break;
        }
        //set vertex BSDF[nE]
        path.m_vert_BSDF_E[nE] = pBsdf;

        //set vertex isDirac[nE]
        path.m_vert_isDirac_E[nE] = pBsdf->isDiracDistribution();

        //if this bounce is a light
        if(intersection.m_pShape->isLight()){
            if(numBounces == 0){
                result += intersection.m_pMaterial->emittance();
            }
            //let's just pretend this never happened...
            break;
        }

        //Sample the BSDF to find the direction the next leg of the path goes in
        float bsdfSampleU, bsdfSampleV;
        samplers.m_bounceSamplers[numBounces]->sample2D(pixelSampleIndex, bsdfSampleU, bsdfSampleV);
        Vector incoming;
        float incomingBsdfPdf = 0.0f;
        float incomingBsdfResult = pBsdf->samplePSA(incoming,
                                                   outgoing,
                                                   normal,
                                                   bsdfSampleU,
                                                   bsdfSampleV,
                                                   incomingBsdfPdf);
        //set vertex Fs
        path.m_vert_Fs_E[nE]->set(incomingBsdfResult * matColor * intersection.m_colorModifier);
        //if the BsdfPdf > 0
        if(incomingBsdfPdf > 0.0f){
            //update currentRay with the new direction and origin
            currentRay.m_origin = position;
            currentRay.m_direction = -incoming;
            currentRay.m_tMax = kRayTMax;
        }
        //else
        else{
            //we're done here, so break
            break;
        }
        //set vertex PdfPsa_E[nE]
        path.m_PdfPsa_E[nE-1] = incomingBsdfPdf;
        //set eyepath PA[nE-1] to the PdfPSA_E we just calculated * the previous PA_E entry
        path.m_vert_PA_E[nE-1] = path.m_PdfPsa_E[nE-1];
        if(nE > 1){
            path.m_vert_PA_E[nE-1] *= path.m_vert_PA_E[nE-2];
        }
        //update the eye throughput
        eyeThroughput *= intersection.m_colorModifier * matColor * incomingBsdfResult *
                        (std::fabs(dot(-incoming, normal)) /
                         (incomingBsdfPdf * bsdfWeight));
        //increment nE
        nE++;
        //increment numBounces
        numBounces++;
    }

    //If we hit the background
    if(numBounces == 0){
        //return black
        return result;
    }

    //If we already found a light
    //should we pretend it didn't happen?
    //or should we just skip the creation of a light path?

    //BUILD A PATH STARTING FROM THE LIGHT
    //initialize the light throughput (for russian roulette) to (1,1,1)
    Color lightThroughput = Color(1.0f, 1.0f, 1.0f);
    //Pick a random light and get a sample from it (a position and direction)
    unsigned int finalBDLightSampleIndex = pixelSampleIndex;
    float bd_liu = samplers.m_BDlightSelectionSampler->sample1D(finalBDLightSampleIndex);
    size_t bd_lightIndex = (size_t)(bd_liu * lights.size());
    if(bd_lightIndex >= lights.size())
        bd_lightIndex = lights.size() - 1;
    Light *pBDLightShape = (Light*) lights[bd_lightIndex];
    //ask the light for a random position
    float bd_leu = samplers.m_BDlightElementSampler->sample1D(finalBDLightSampleIndex);
    float bd_lsu, bd_lsv;
    samplers.m_BDlightSampler->sample2D(finalBDLightSampleIndex, bd_lsu, bd_lsv);
    Point bd_lightPoint;
    Vector bd_lightNormal;
    float dummyLightPdf = 0.0f; //dummy value?
    Point dummyPosition = Point();
    Vector dummyNormal = Vector(1.0f, 1.0f, 1.0f);
    pBDLightShape->sampleSurface(dummyPosition,
                                 dummyNormal,
                                 ray.m_time,
                                 bd_lsu, bd_lsv, bd_leu,
                                 bd_lightPoint,
                                 bd_lightNormal,
                                 dummyLightPdf);
    //Pick a random direction in the hemisphere (we're assuming the light emits diffusely)
    float bd_ldu, bd_ldv;
    samplers.m_BDlightDirectionSampler->sample2D(finalBDLightSampleIndex, bd_ldu, bd_ldv);
    Vector bd_localOutgoing = uniformToCosineHemisphere(bd_ldu, bd_ldv);
    Vector x, y, z;
    makeCoordinateSpace(bd_lightNormal, x, y, z);
    Vector bd_outgoing = transformFromLocalCoordinateSpace(bd_localOutgoing, x, y, z);
    if(dot(bd_outgoing, bd_lightNormal) < 0.0f)
        bd_outgoing *= -1.0f;

    //create a ray using the light's position and direction
    //set up the first ray starting from the light
    currentRay.m_origin = bd_lightPoint;
    currentRay.m_direction = bd_outgoing;
    currentRay.m_tMax = kRayTMax;

    //Calculate PA(y0)
    float PAy0 = pBDLightShape->surfaceAreaPdf();
    //set the last vert_position to the light's position
    path.m_vert_position_L[0]->set(currentRay.m_origin);
    //set the first raydir
    path.m_outdir_L[0]->set(currentRay.m_direction);
    //set the first PDFPSA_L
    path.m_PdfPsa_L[0] = 1.0f / M_PI;   //TODO: is this right?

    //set the first vert_PA_L
    //path.m_vert_PA_L[0] = probability of selecting the light
    path.m_vert_PA_L[0] = PAy0;

    //initialize the light vertex count, nL, to 1
    size_t nL = 1;
    numBounces = 0;
    //while depth < max depth
    while(numBounces < samplers.m_maxRayDepth){
        //if numbounces > min depth, do russian roulette
        if(false && numBounces >= samplers.m_minRayDepth){
            //get the maximum component of the light throughput
            float maxComp = std::max(std::max(lightThroughput.m_r, lightThroughput.m_g), lightThroughput.m_b);
            //get a random number
            float rrSample = samplers.m_lightpathRussianRouletteSamplers[numBounces - samplers.m_minRayDepth]->sample1D(pixelSampleIndex);
            //if the random number is greater than the max component
            if(rrSample > maxComp){
                //you've been killed
                break;
            }
        }
        //Trace the ray to see if we hit anything
        Intersection intersection(currentRay);
        //if we didn't
        if(!scene.intersect(intersection)){
            //we can still use this light sample, so we're not done with this path
            //break, but don't return
            break;
        }
        //evaluate the material and intersection information at this bounce
        //set lightpath vertex position [nL]
        Point position = *path.m_vert_position_L[nL] = intersection.position();
        //set lightpath vertex normal[nL]
        Vector normal = *path.m_vert_normal_L[nL] = intersection.m_normal;
        //set outdir_L[nL-1]
        Vector outgoing = *path.m_outdir_L[nL] = currentRay.m_direction;    //TODO: should this be -m_direction?
        Bsdf* pBsdf = NULL;
        float bsdfWeight = 1.0f;
        Color matColor = *path.m_vert_matColor_L[nL] = intersection.m_pMaterial->evaluate(position,
                                                            normal,
                                                            outgoing,   //TODO: should this be -outgoing?
                                                            pBsdf,
                                                            bsdfWeight);
        // No BSDF? We can't evaluate lighting, so bail.
        if(pBsdf == NULL){
            break;
        }
        //set lightpath vertex BSDF[nL]
        path.m_vert_BSDF_L[nL] = pBsdf;

        //set lightpath vertex isDirac[nL]
        path.m_vert_isDirac_L[nL] = pBsdf->isDiracDistribution();


        //if this bounce is a light
        if(intersection.m_pShape->isLight()){
            //let's pretend it never happened and end the light path here
            break;
        }

        //Sample the BSDF to find the direction the next leg of the path goes in
        float bsdfSampleU, bsdfSampleV;
        samplers.m_lightBounceSamplers[numBounces]->sample2D(pixelSampleIndex, bsdfSampleU, bsdfSampleV);
        Vector incoming;
        float incomingBsdfPdf = 0.0f;
        float incomingBsdfResult = pBsdf->samplePSA(incoming,
                                                    outgoing,   //TODO: should this be -outgoing?
                                                    normal,
                                                    bsdfSampleU,
                                                    bsdfSampleV,
                                                    incomingBsdfPdf);

        //set vertex Fs
        path.m_vert_Fs_L[nL]->set(incomingBsdfResult * matColor * intersection.m_colorModifier);
        //if the BsdfPdf > 0
        if(incomingBsdfPdf > 0.0f){
            //update currentRay with the new direction and origin
            currentRay.m_origin = position;
            currentRay.m_direction = -incoming;
            currentRay.m_tMax = kRayTMax;
        }
        //else
        else{
            //we're done here, so break
            break;
        }
        //set vertex PdfPsa[nL]
        path.m_PdfPsa_L[nL] = incomingBsdfPdf;
        //set lightpath PA[nL] to the PdfPSA_L we just calculated * the previous PA_L entry
        path.m_vert_PA_L[nL] = path.m_PdfPsa_L[nL] * path.m_vert_PA_L[nL-1];

        //update the light throughput
        lightThroughput *= intersection.m_colorModifier * matColor * incomingBsdfResult *
                            (std::fabs(dot(-incoming, normal)) /
                             (incomingBsdfPdf * bsdfWeight));
        //increment nL
        nL++;
        //increment numBounces
        numBounces++;
    }

    //BUILD VARIATIONS OF THE COMBINED PATHS
    float subpathWeight = 1.0f / (float)(nL * (nE-1));
    //for all vertices in the lightpath
    for(size_t subL = 1; subL <= nL; subL++){
        //for all vertices in the eyepath (except we don't want to start with only the lens so we only contribute to this pixel)
        for(size_t subE = 2; subE <= nE; subE++){
        //SUM THEIR CONTRIBUTIONS ACCORDING TO THEIR WEIGHTS AND PROBABILITY DISTRIBUTIONS
            //get the indices for vertices subL and subE
            size_t lVertIdx = subL - 1;
            size_t eVertIdx = subE - 1;
            //if vertex L or vertex E is a Dirac distribution
            if(path.m_vert_isDirac_E[eVertIdx] || path.m_vert_isDirac_L[lVertIdx]){
                //skip this path
                continue;
            }
            Vector LtoE = *path.m_vert_position_E[eVertIdx] - *path.m_vert_position_L[lVertIdx];
            Vector LtoEDir = LtoE.normalized();
            //calculate the Pdf for connecting from vertex subL to subE
            float connectPdf_L = 0.0f;
            float connectingBsdfResult_L;
            //if it's just the light
            if(subL == 1){
                //we'll just assume a constant PDF
                connectingBsdfResult_L = 1.0f / M_PI;    //TODO: Is this right?
            }
            else{
                connectingBsdfResult_L = path.m_vert_BSDF_L[lVertIdx]->evaluatePSA(*path.m_outdir_L[lVertIdx-1],
                                                                                LtoEDir,
                                                                                *path.m_vert_normal_L[lVertIdx],
                                                                                connectPdf_L);
            }
            //if it's 0
            if(connectPdf_L == 0.0f || connectingBsdfResult_L == 0.0f){
                //skip this path
                continue;
            }
            //calculate the Pdf for subE to subE-1 with the new incoming direction
            float connectPdf_E = 0.0f;
            float connectingBsdfResult_E = path.m_vert_BSDF_E[eVertIdx]->evaluatePSA(LtoEDir,
                                                                                *path.m_outdir_E[eVertIdx-1],
                                                                                *path.m_vert_normal_E[eVertIdx],
                                                                                connectPdf_E);
            //if it's 0
            if(connectPdf_E == 0.0f || connectingBsdfResult_E == 0.0f){
                continue;
            }
            //calculate the visibility term for the connecting edge
            Ray connectRay = Ray(*path.m_vert_position_L[lVertIdx], LtoEDir, LtoE.length(), ray.m_time);
            Intersection connectIntersection(connectRay);
            //if it's 0
            if(scene.intersect(connectIntersection)){
                //skip this path
                continue;
            }

            // Calculate the probability of this path being sampled, pst;
            //pst = vert_PA_L[subL-1] * vert_PA_E[subE-1] * connectPdf
            float pst = path.m_vert_PA_L[lVertIdx] * path.m_vert_PA_E[eVertIdx-1] * connectPdf_L * connectPdf_E;
            //pst = path.m_vert_PA_L[lVertIdx] * connectPdf_L * connectPdf_E;

            //create a lightpath BSDF value, Fs_L, set to (1,1,1)
            Color Fs_L = Color(1.0f, 1.0f, 1.0f);
            //create an eyepath BSDF value, Fs_E, set to (1,1,1)
            Color Fs_E = Color(1.0f, 1.0f, 1.0f);
            //for each vertex on the light path (except the connecting vertex and the first vertex)
            for(size_t idx = 1; idx < lVertIdx; idx++){
                //Fs_L *= vert_Fs_L[vert index]
                Fs_L *= *path.m_vert_Fs_L[idx];
            }
            //for each vertex on the eye path (except the connecting vertex and the first vertex)
            for(size_t idx = 1; idx < eVertIdx; idx++){
                //Fs_E *= vert_Fs_E[vert index]
                Fs_E *= *path.m_vert_Fs_E[idx];
            }
            //get the BSDF value for the lightpath's connecting vertex, Fs_L_connect
            //Color Fs_L_connect = vertex subL mat color * connectingBsdfResult_L
            Color Fs_L_connect = *path.m_vert_matColor_L[lVertIdx] * connectingBsdfResult_L;
            //get the BSDF value for the eyepath's connecting vertex, Fs_E_connect
            //Color Fs_E_connect = vertex subE mat color * connectingBsdfResult_E
            Color Fs_E_connect = *path.m_vert_matColor_E[eVertIdx] * connectingBsdfResult_E;
            //the final value of this subpath
            //subpath = light color * light power * Fs_L * Fs_E * Fs_L_connect * Fs_E_connect
            Color subpathResult = pBDLightShape->emitted() * Fs_L * Fs_E * Fs_L_connect * Fs_E_connect;
            //add the weighted contribution to the result
            result += (subpathResult / pst) * subpathWeight;
        }
    }
    //return the result
    return result;
}
*/
//somewhat naive bidirectional pathTrace function
Color BDpathTrace(const Ray& ray,
                  ShapeSet& scene,
                  std::vector<Shape*>& lights,
                  Rng& rng,
                  SamplerContainer& samplers,
                  PathVertexContainer& path,
                  unsigned int pixelSampleIndex){
    // Accumulate total incoming radiance in 'result'
    Color result = Color(0.0f, 0.0f, 0.0f);

    // As we get through more and more bounces, we track how much the light is
    // diminished through each bounce
    Color throughput = Color(1.0f, 1.0f, 1.0f);

    //BUILD A PATH STARTING FROM THE CAMERA

    // Start with the initial ray from the camera
    Ray currentRay = ray;

    //set vert_position_E[0] to the camera's position
    *path.m_vert_position_E[0] = currentRay.m_origin;

    //We don't really need to worry about vert_normal[0], vert_isDirac[0], vert_BSDF[0], vert_matColor[0], or vert_BSDF_weight[0]

    // While we have bounces left we can still take...
    size_t numBounces = 0;
    size_t nE = 1;
    bool lastBounceDiracDistribution = false;
    while(numBounces < samplers.m_maxRayDepth){
        //If we've gotten past our minimum depth, do Russian Roulette
        if(numBounces > samplers.m_minRayDepth){
            //get the maximum component of the eye throughput
            float maxComp = std::max(std::max(throughput.m_r, throughput.m_g), throughput.m_b);
            //get a random number
            float rrSample = samplers.m_eyepathRussianRouletteSamplers[numBounces - samplers.m_minRayDepth]->sample1D(pixelSampleIndex);
            //if the random number is greater than the max component
            if(rrSample > maxComp){
                //kill the path here
                break;
            }
        }
        // Trace the ray to see if we hit anything
        Intersection intersection(currentRay);
        //if not
        if(!scene.intersect(intersection)){
            //return background
            break;
        }
        // Add in emission where directly visible from the camera or if the
        // last bounce was pure specular, as this is the only way to account
        // for the light. (This light won't be picked up by the path combiner).
        if(numBounces == 0 || lastBounceDiracDistribution){
            result += throughput * intersection.m_pMaterial->emittance();
        }
        // Evaluate the material and intersection information at this bounce
        //store the position
        Point position = *path.m_vert_position_E[nE] = intersection.position();
        //store the normal
        Vector normal = *path.m_vert_normal_E[nE] = intersection.m_normal;
        //store the outgoing direction
        Vector outgoing = *path.m_vert_outdir_E[nE] = -currentRay.m_direction;
        //store the Bsdf
        Bsdf* pBsdf = NULL;
        float bsdfWeight = 1.0f;
        //store the material color and color modifier
        Color matColor = *path.m_vert_matColor_E[nE] = intersection.m_pMaterial->evaluate(position,
                                                                          normal,
                                                                          outgoing,
                                                                          pBsdf,
                                                                          bsdfWeight);
        path.m_vert_BSDF_E[nE] = pBsdf;
        *path.m_vert_colorModifier_E[nE] = intersection.m_colorModifier;
        path.m_vert_pShape_E[nE] = intersection.m_pShape;
        path.m_vert_pMaterial_E[nE] = intersection.m_pMaterial;
        //store the bsdf weight
        path.m_vert_BSDF_weight_E[nE] = bsdfWeight;
        // No BSDF? We can't evaluate lighting, so bail
        if(pBsdf == NULL){
            break;
        }
        // Was this a perfect specular bounce? (store it)
        lastBounceDiracDistribution = path.m_vert_isDirac_E[nE] = pBsdf->isDiracDistribution();
        // Sample the BSDF to find the direction the next leg of the path goes in
        float bsdfSampleU, bsdfSampleV;
        samplers.m_bounceSamplers[numBounces]->sample2D(pixelSampleIndex, bsdfSampleU, bsdfSampleV);
        Vector incoming;
        float incomingBsdfPdf = 0.0f;
        float incomingBsdfResult = pBsdf->sampleSA(incoming,
                                                   outgoing,
                                                   normal,
                                                   bsdfSampleU,
                                                   bsdfSampleV,
                                                   incomingBsdfPdf);
        //store the Pdf value for this bounce (VERY important for Perfect Reflection BRDFs)
        path.m_PdfSa_E[nE] = incomingBsdfPdf;
        //store the BsdfResult so we don't have to recalculate it later
        path.m_vert_BSDF_result_E[nE] = incomingBsdfResult;
        if(incomingBsdfPdf > 0.0f){
            //setup the currentRay for the next bounce
            currentRay.m_origin = position;
            currentRay.m_direction = -incoming;
            currentRay.m_tMax = kRayTMax;
            // Reduce lighting effect for the next bounce based on this bounce's BSDF
            throughput *= intersection.m_colorModifier * matColor * incomingBsdfResult *
                    (std::fabs(dot(-incoming, normal)) /
                     (incomingBsdfPdf * bsdfWeight));
        }
        else{
            break;  //BSDF is zero, stop bouncing
        }

        numBounces++;
        nE++;
    }

    //if we hit the background first
    if(numBounces == 0){
        //just return black
        return result;
    }

    //BUILD A PATH STARTING FROM THE LIGHT
    // Keep track of the lightpath throughput? (it'd only be useful for russian roulette)
    //Pick a random light and get a sample from it (position and direction)
    unsigned int finalBDLightSampleIndex = pixelSampleIndex;
    float bd_liu = samplers.m_BDlightSelectionSampler->sample1D(finalBDLightSampleIndex);
    size_t bd_lightIndex = (size_t)(bd_liu * lights.size());
    if(bd_lightIndex >= lights.size())
        bd_lightIndex = lights.size() - 1;
    Light *pBDLightShape = (Light*)lights[bd_lightIndex];
    //ask the light for a random position
    float bd_leu = samplers.m_BDlightElementSampler->sample1D(finalBDLightSampleIndex);
    float bd_lsu, bd_lsv;
    samplers.m_BDlightSampler->sample2D(finalBDLightSampleIndex, bd_lsu, bd_lsv);
    Point bd_lightPoint;
    Vector bd_lightNormal;
    float dummyLightPdf = 0.0f; //dummy value?
    Point dummyPosition = Point();
    Vector dummyNormal = Vector(1.0f, 1.0f, 1.0f);
    pBDLightShape->sampleSurface(dummyPosition,
                                 dummyNormal,
                                 ray.m_time,
                                 bd_lsu, bd_lsv, bd_leu,
                                 bd_lightPoint,
                                 bd_lightNormal,
                                 dummyLightPdf);
    //Pick a random direction in the hemisphere (we're assuming the light emits diffusely)
    float bd_ldu, bd_ldv;
    samplers.m_BDlightDirectionSampler->sample2D(finalBDLightSampleIndex, bd_ldu, bd_ldv);
    Vector bd_localOutgoing = uniformToCosineHemisphere(bd_ldu, bd_ldv);
    Vector x, y, z;
    makeCoordinateSpace(bd_lightNormal, x, y, z);
    Vector bd_outgoing = transformFromLocalCoordinateSpace(bd_localOutgoing, x, y, z);
    if(dot(bd_outgoing, bd_lightNormal) < 0.0f)
        bd_outgoing *= -1.0f;

    *path.m_vert_position_L[0] = bd_lightPoint;
    path.m_vert_pShape_L[0] = pBDLightShape;
    //store the initial outgoing direction
    *path.m_vert_outdir_L[0] = bd_outgoing;

    //Create a ray using the light's position and direction
    currentRay.m_origin = bd_lightPoint;
    currentRay.m_direction = bd_outgoing;
    currentRay.m_tMax = kRayTMax;

    //reset the throughput
    throughput = Color(1.0f, 1.0f, 1.0f);

    numBounces = 0;
    size_t nL = 1;
    //While we have light bounces left
    while(numBounces < samplers.m_maxRayDepth){
        //If we've gotten past our minimum depth, do Russian Roulette
        if(numBounces > samplers.m_minRayDepth){
            //get the maximum component of the eye throughput
            float maxComp = std::max(std::max(throughput.m_r, throughput.m_g), throughput.m_b);
            //get a random number
            float rrSample = samplers.m_lightpathRussianRouletteSamplers[numBounces - samplers.m_minRayDepth]->sample1D(pixelSampleIndex);
            //if the random number is greater than the max component
            if(rrSample > maxComp){
                //kill the path here
                break;
            }
        }
        //Trace the ray to see if we hit anything
        Intersection intersection(currentRay);
        //if not
        if(!scene.intersect(intersection)){
            //end the lightpath here
            break;
        }
        //Evaluate the material and intersection information at this bounce
        //store the position
        Point position = *path.m_vert_position_L[nL] = intersection.position();
        //store the normal
        Vector normal = *path.m_vert_normal_L[nL] = intersection.m_normal;
        //store the outgoing direction
        Vector outgoing = -currentRay.m_direction;
        //store the Bsdf
        Bsdf* pBsdf = NULL;
        float bsdfWeight = 1.0f;
        //store the material color and color modulator
        Color matColor = *path.m_vert_matColor_L[nL] = intersection.m_pMaterial->evaluate(position,
                                                                          normal,
                                                                          outgoing,
                                                                          pBsdf,
                                                                          bsdfWeight);
        path.m_vert_BSDF_L[nL] = pBsdf;
        *path.m_vert_colorModifier_L[nL] = intersection.m_colorModifier;
        path.m_vert_pShape_L[nL] = intersection.m_pShape;
        path.m_vert_pMaterial_L[nL] = intersection.m_pMaterial;
        //store the Bsdf weight
        path.m_vert_BSDF_weight_L[nL] = bsdfWeight;
        //No BSDF? We can't evaluate lighting, so bail
        if(pBsdf == NULL){
            break;
        }
        //Was this a perfect specular bounce? (store it)
        path.m_vert_isDirac_L[nL] = pBsdf->isDiracDistribution();
        //Sample the BSDF to find the direction of the next leg of the path
        float bsdfSampleU, bsdfSampleV;
        samplers.m_lightBounceSamplers[numBounces]->sample2D(pixelSampleIndex, bsdfSampleU, bsdfSampleV);
        Vector incoming;
        float incomingBsdfPdf = 0.0f;
        float incomingBsdfResult = pBsdf->sampleSA(incoming,
                                                   outgoing,
                                                   normal,
                                                   bsdfSampleU,
                                                   bsdfSampleV,
                                                   incomingBsdfPdf);
        //store the Pdf value for this bounce (VERY important for Perfect Reflection BRDFs)
        path.m_PdfSa_L[nL] = incomingBsdfPdf;
        //store the BsdfResult so we don't have to recalculate it later
        path.m_vert_BSDF_result_L[nL] = incomingBsdfResult;
        if(incomingBsdfPdf > 0.0f){
            //setup the currentRay for the next bounce
            currentRay.m_origin = position;
            currentRay.m_direction = -incoming;
            currentRay.m_tMax = kRayTMax;
            //store "-incoming" as this vertex's outgoing direction
            *path.m_vert_outdir_L[nL] = -incoming;
            // Reduce lighting effect for the next bounce based on this bounce's BSDF
            throughput *= intersection.m_colorModifier * matColor * incomingBsdfResult *
                    (std::fabs(dot(-incoming, normal)) /
                     (incomingBsdfPdf * bsdfWeight));
        }
        else{
            break;  //BSDF is zero, stop bouncing
        }

        numBounces++;
        nL++;
    }

    //COMBINE THE PATHS AND ACCUMULATE LIGHT
    //Calculate a weighting factor for each subpath
    float subpathWeight = 1.0f / (float)(nL * (nE-1));
    //For each possible lightpath length
    for(size_t subL = 1; subL <= nL; subL++){
        //For each possible eyepath length (minimum of 2 vertices)
        for(size_t subE = 2; subE <= nE; subE++){
            size_t lVertIdx = subL-1;
            size_t eVertIdx = subE-1;
            size_t numVerts = subL + subE;
            //size_t numEdges = numVerts - 1;
            //If either connecting vertex is a dirac distribution
            if(path.m_vert_isDirac_E[eVertIdx] || path.m_vert_isDirac_L[lVertIdx]){
                //skip this subpath
                continue;
            }
            //Get the vector from the end of the lightpath to the end of the eyepath
            Vector LtoE = *path.m_vert_position_E[eVertIdx] - *path.m_vert_position_L[lVertIdx];
            Vector LtoEDir = LtoE.normalized();
            //If the connecting vertices aren't mutually visible
            //(BRDFs where dot(normal, outgoing) < 0)?
            if(path.m_vert_BSDF_E[eVertIdx]->type() == IS_BRDF){
                if(dot(*path.m_vert_normal_E[eVertIdx], LtoEDir) > 0.0f){
                    continue;
                }
            }
            if(lVertIdx > 0 && path.m_vert_BSDF_L[lVertIdx]->type() == IS_BRDF){
                if(dot(*path.m_vert_normal_L[lVertIdx], LtoEDir) < 0.0f){
                    continue;
                }
            }
            //(BTDFs where dot(normal, outgoing) > 0)?  //TODO?
            //I think we have to make sure that outgoing and incoming are on different hemispheres for each point
            //TODO

            Ray connectRay = Ray(*path.m_vert_position_L[lVertIdx], LtoEDir, LtoE.length(), ray.m_time);
            Intersection connectIntersection(connectRay);
            //(There's something in between the surfaces)
            if(scene.intersect(connectIntersection)){
                //skip this subpath
                continue;
            }
            // As we get through more and more bounces, we track how much the light is
            // diminished through each bounce
            //Initialize the throughput to ones
            throughput = Color(1.0f, 1.0f, 1.0f);
            //Initialize the subpathResult to zeros
            Color subpathResult = Color(0.0f, 0.0f, 0.0f);
            //Initialize numDiracBounces to zero
            size_t numDiracBounces = 0;
            numBounces = 0;
            lastBounceDiracDistribution = false;
            //setup the lastPoint for calculating outgoing direction
            Point prevPosition = *path.m_vert_position_E[0];
            //For each bounce in this subpath
            for(size_t v_idx = 1; v_idx < numVerts; v_idx++){
                bool vertInEyepath = v_idx <= eVertIdx;
                //if this vertex is one of the connecting vertices, we need to know
                bool isEyeConnectingVert = v_idx == eVertIdx;
                bool isLightConnectingVert = v_idx == eVertIdx + 1;
                bool vertIsConnectingVert = isEyeConnectingVert || isLightConnectingVert;
                size_t local_v_idx = vertInEyepath? v_idx : lVertIdx - (v_idx - eVertIdx - 1);
                //get the next point in the path (position, normal, outgoing, BSDF, matColor, bsdfWeight, isDirac)
                Point position;
                Vector normal;
                Bsdf* pBsdf;
                Shape* pShape;
                Color matColor;
                Color colorModifier;
                Material* pMaterial;
                float bsdfWeight;
                bool isDirac;
                float incomingBsdfPdf;
                float incomingBsdfResult;
                Vector outgoing;
                Vector incoming;
                if(vertInEyepath){
                    position =          *path.m_vert_position_E[local_v_idx];
                    normal =            *path.m_vert_normal_E[local_v_idx];
                    pBsdf =             path.m_vert_BSDF_E[local_v_idx];
                    pShape =            path.m_vert_pShape_E[local_v_idx];
                    matColor =          *path.m_vert_matColor_E[local_v_idx];
                    colorModifier =     *path.m_vert_colorModifier_E[local_v_idx];
                    pMaterial =         path.m_vert_pMaterial_E[local_v_idx];
                    bsdfWeight =        path.m_vert_BSDF_weight_E[local_v_idx];
                    isDirac =           path.m_vert_isDirac_E[local_v_idx];
                    incomingBsdfPdf =   path.m_PdfSa_E[local_v_idx];
                    incomingBsdfResult =path.m_vert_BSDF_result_E[local_v_idx];
                    outgoing =          *path.m_vert_outdir_E[local_v_idx];
                }
                else{
                    position =          *path.m_vert_position_L[local_v_idx];
                    normal =            *path.m_vert_normal_L[local_v_idx];
                    pBsdf =             path.m_vert_BSDF_L[local_v_idx];
                    pShape =            path.m_vert_pShape_L[local_v_idx];
                    matColor =          *path.m_vert_matColor_L[local_v_idx];
                    colorModifier =     *path.m_vert_colorModifier_L[local_v_idx];
                    pMaterial =         path.m_vert_pMaterial_L[local_v_idx];
                    bsdfWeight =        path.m_vert_BSDF_weight_L[local_v_idx];
                    isDirac =           path.m_vert_isDirac_L[local_v_idx];
                    incomingBsdfPdf =   path.m_PdfSa_L[local_v_idx];
                    incomingBsdfResult =path.m_vert_BSDF_result_L[local_v_idx];
                    outgoing =          *path.m_vert_outdir_L[local_v_idx];
                }
                //if this is the lightpath connecting vertex
                if(isLightConnectingVert){
                    //the outgoing direction is LtoEDir
                    outgoing = LtoEDir;
                }
                // No BSDF? We can't evaluate lighting, so bail
                if(pBsdf == NULL){
                    break;
                }

                // Add in emission when directly visible from the camera or if the
                // last bounce was pure specular, as this is the only way to account
                // for the light. Also if the encountered surface is *not* a light, we
                // should always add its emission.
                if(!pShape->isLight() || numBounces == 0 || lastBounceDiracDistribution){
                    subpathResult += throughput * pMaterial->emittance();
                }
                // Was this a perfect specular bounce?
                lastBounceDiracDistribution = isDirac;
                if(lastBounceDiracDistribution){
                    numDiracBounces++;
                }

                //Evaluate direct lighting
                if(!lastBounceDiracDistribution){
                    Color lightResult = Color(0.0f, 0.0f, 0.0f);
                    float lightSelectionWeight = float(lights.size()) / samplers.m_numLightSamples;
                    for(size_t lightSampleIndex = 0; lightSampleIndex < samplers.m_numLightSamples; ++lightSampleIndex){
                        // Sample lights using MIS between the light and the BSDF.
                        // This means we ask the light for a direction, and the likelihood
                        // of having sampled that direction (the PDF).  Then we ask the
                        // BSDF what it thinks of that direction (its PDF), and weight
                        // the light sample with MIS.
                        //
                        // Then, we ask the BSDF for a direction, and the likelihood of
                        // having sampled that direction (the PDF).  Then we ask the
                        // light what it thinks of that direction (its PDF, and whether
                        // that direction even runs into the light at all), and weight
                        // the BSDF sample with MIS.
                        //
                        // By doing both samples and asking both the BSDF and light for
                        // their PDF for each one, we can combine the strengths of both
                        // sampling methods and get the best of both worlds.  It does
                        // cost an extra shadow ray and evaluation, though, but it is
                        // generally such an improvement in quality that it is very much
                        // worth the overhead.

                        // Select a light randomly for this sample
                        unsigned int finalLightSampleIndex = pixelSampleIndex * samplers.m_numLightSamples +
                                                            lightSampleIndex;
                        float liu = samplers.m_lightSelectionSamplers[numBounces]->sample1D(finalLightSampleIndex);
                        size_t lightIndex = (size_t)(liu * lights.size());
                        if(lightIndex >= lights.size())
                            lightIndex = lights.size() - 1;
                        Light *pLightShape = (Light*) lights[lightIndex];

                        // Ask the light for a random position/normal we can use for lighting
                        float lsu, lsv;
                        samplers.m_lightSamplers[numBounces]->sample2D(finalLightSampleIndex, lsu, lsv);
                        float leu = samplers.m_lightElementSamplers[numBounces]->sample1D(finalLightSampleIndex);
                        Point lightPoint;
                        Vector lightNormal;
                        float lightPdf = 0.0f;
                        pLightShape->sampleSurface(position,
                                                   normal,
                                                   ray.m_time,
                                                   lsu, lsv, leu,
                                                   lightPoint,
                                                   lightNormal,
                                                   lightPdf);

                        if (lightPdf > 0.0f)
                        {
                            // Ask the BSDF what it thinks of this light position (for MIS)
                            Vector lightIncoming = position - lightPoint;
                            float lightDistance = lightIncoming.normalize();
                            float bsdfPdf = 0.0f;
                            float bsdfResult = pBsdf->evaluateSA(lightIncoming,
                                                                 outgoing,
                                                                 normal,
                                                                 bsdfPdf);
                            if (bsdfResult > 0.0f && bsdfPdf > 0.0f)
                            {
                                // Fire a shadow ray to make sure we can actually see the light position
                                Ray shadowRay(position, -lightIncoming, lightDistance - kRayTMin, ray.m_time);
                                if (!scene.doesIntersect(shadowRay))
                                {
                                    // The light point is visible, so let's add that
                                    // contribution (mixed by MIS)
                                    float misWeightLight = powerHeuristic(1, lightPdf, 1, bsdfPdf);
                                    lightResult += pLightShape->emitted() *
                                                   colorModifier * matColor *
                                                   bsdfResult *
                                                   std::fabs(dot(-lightIncoming, normal)) *
                                                   misWeightLight / (lightPdf * bsdfWeight);
                                }
                            }
                        }

                        // Ask the BSDF for a sample direction
                        float bsu, bsv;
                        samplers.m_bsdfSamplers[numBounces]->sample2D(finalLightSampleIndex, bsu, bsv);
                        Vector bsdfIncoming;
                        float bsdfPdf = 0.0f;
                        float bsdfResult = pBsdf->sampleSA(bsdfIncoming,
                                                           outgoing,
                                                           normal,
                                                           bsu,
                                                           bsv,
                                                           bsdfPdf);
                        if (bsdfPdf > 0.0f && bsdfResult > 0.0f)
                        {
                            Intersection shadowIntersection(Ray(position, -bsdfIncoming, kRayTMax, ray.m_time));
                            bool intersected = scene.intersect(shadowIntersection);
                            if (intersected && shadowIntersection.m_pShape == pLightShape)
                            {
                                // Ask the light what it thinks of this direction (for MIS)
                                lightPdf = pLightShape->intersectPdf(shadowIntersection);
                                if (lightPdf > 0.0f)
                                {
                                    // BSDF chose the light, so let's add that
                                    // contribution (mixed by MIS)
                                    float misWeightBsdf = powerHeuristic(1, bsdfPdf, 1, lightPdf);
                                    lightResult += pLightShape->emitted() *
                                                   colorModifier * matColor * bsdfResult *
                                                   std::fabs(dot(-bsdfIncoming, normal)) * misWeightBsdf /
                                                   (bsdfPdf * bsdfWeight);
                                }
                            }
                        }
                    }

                    // Average light samples
                    lightResult *= samplers.m_numLightSamples > 0 ? lightSelectionWeight : 0.0f;

                    // Add direct lighting at this bounce (modified by how much the
                    // previous bounces have dimmed it)
                    subpathResult += throughput * lightResult;
                }

                // Get the next position
                Point nextPosition;
                bool nextVertInEyepath = v_idx + 1 <= eVertIdx;
                size_t local_next_v_idx = nextVertInEyepath? v_idx + 1 : lVertIdx - (v_idx - eVertIdx);
                //if this is the eyepath connecting vertex
                if(isEyeConnectingVert){
                    //incoming direction is LtoEDir
                    incoming = LtoEDir;
                }
                //if this and the next vertex are in the eyepath
                else if(vertInEyepath && nextVertInEyepath){
                    //nextPosition = *path.m_vert_position_E[local_next_v_idx];
                    //incoming is the outgoing direction of the next vertex
                    incoming = *path.m_vert_outdir_E[local_next_v_idx];
                }
                //otherwise, it's in the lightpath
                else{
                    //nextPosition = *path.m_vert_position_L[local_next_v_idx];
                    //its incoming direction is simply the next vertex's outgoing direction
                    incoming = *path.m_vert_outdir_L[local_next_v_idx];
                }
                // Get the incoming direction
                //Vector incoming = (position - nextPosition).normalized();    //pointing toward the surface, not away from it

                //if this is a connecting vertex
                //(if it's not, we've already grabbed the correct values above)
                if(vertIsConnectingVert){
                    //Evaluate material for actual outgoing and incoming directions
                    //Get the IncomingBsdfPdf and IncomingBsdfResult for the next leg of the path
                    incomingBsdfPdf = 0.0f;
                    incomingBsdfResult = pBsdf->evaluateSA(incoming,
                                                             outgoing,
                                                             normal,
                                                             incomingBsdfPdf);
                }

                //if incomingBsdfPdf is greater than 0
                if(incomingBsdfPdf > 0.0f){
                    //calculate the geometric term for this edge
                    float geometricTerm = 1.0f;
                    //if this is the connecting edge
                    /*if(isEyeConnectingVert){
                        //geometric term = 1/squared length of connecting edge
                        geometricTerm = 1.0f / LtoE.length2();
                        //Do we have to worry about angles between normals and incoming/outgoing directions?
                    }*/

                    // Reduce lighting effect for the next bounce based on this bounce's BSDF
                    throughput *= colorModifier * matColor * incomingBsdfResult *
                            geometricTerm *
                            (std::fabs(dot(-incoming, normal)) /
                             (incomingBsdfPdf * bsdfWeight));
                }
                //else
                else{
                    //break; we're done here
                    break;
                }
                numBounces++;
                prevPosition = position;
            }
            result += subpathResult * subpathWeight;
        }
    }

    //This represents an estimate of the total light coming in along the path
    return result;
}

Image* raytrace(ShapeSet& scene,
                const Camera& cam,
                size_t width,
                size_t height,
                unsigned int pixelSamplesHint,
                unsigned int lightSamplesHint,
                unsigned int maxRayDepth)
{
    // Get light list from the scene
    std::vector<Shape*> lights;
    scene.findLights(lights);
    
    scene.prepare();
    
    // Set up the output image
    Image *pImage = new Image(width, height);
    
    // Set up render threads; we make as much as 16 chunks of the image that
    // can render in parallel.
    const size_t kChunkDim = 4;
    
    // Chunk size is the number of pixels per image chunk (we have to take care
    // to deal with tiny images)
    size_t xChunkSize = width >= kChunkDim ? width / kChunkDim : 1;
    size_t yChunkSize = height >= kChunkDim ? height / kChunkDim : 1;
    // Chunks are the number of chunks in each dimension we can chop the image
    // into (again, taking care to deal with tiny images, and also images that
    // don't divide clealy into 4 chunks)
    size_t xChunks = width > kChunkDim ? width / xChunkSize : 1;
    size_t yChunks = height > kChunkDim ? height / yChunkSize : 1;
    if (xChunks * xChunkSize < width) xChunks++;
    if (yChunks * yChunkSize < height) yChunks++;
    
    // Set up render threads
    size_t numRenderThreads = xChunks * yChunks;
    RenderThread **renderThreads = new RenderThread*[numRenderThreads];
    
    // Launch render threads
    for (size_t yc = 0; yc < yChunks; ++yc)
    {
        // Get the row start/end (making sure the last chunk doesn't go off the end)
        size_t yStart = yc * yChunkSize;
        size_t yEnd = std::min((yc + 1) * yChunkSize, height);
        for (size_t xc = 0; xc < xChunks; ++xc)
        {
            // Get the column start/end (making sure the last chunk doesn't go off the end)
            size_t xStart = xc * xChunkSize;
            size_t xEnd = std::min((xc + 1) * xChunkSize, width);
            // Render the chunk!
            renderThreads[yc * xChunks + xc] = new RenderThread(xStart,
                                                                xEnd,
                                                                yStart,
                                                                yEnd,
                                                                pImage,
                                                                scene,
                                                                cam,
                                                                lights,
                                                                pixelSamplesHint,
                                                                lightSamplesHint,
                                                                maxRayDepth);
            renderThreads[yc * xChunks + xc]->start();
        }
    }
    
    // Wait until the render finishes
    bool stillRunning;
    do
    {
        // See if any render thread is still going...
        stillRunning = false;
        for (size_t i = 0; i < numRenderThreads; ++i)
        {
            if (renderThreads[i]->isRunning())
            {
                stillRunning = true;
                break;
            }
        }
        if (stillRunning)
        {
            // Give up the CPU so the render threads can do their thing
            QThread::yieldCurrentThread();
        }
    } while (stillRunning);
    
    // Clean up render thread objects
    for (size_t i = 0; i < numRenderThreads; ++i)
    {
        delete renderThreads[i];
    }
    delete[] renderThreads;
    
    // We made a picture!
    return pImage;
}


} // namespace Rayito
