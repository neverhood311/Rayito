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
            samplers.m_lightBounceSamplers.push_back(new CorrelatedMultiJitterSampler(m_pixelSamplesHint,
                                                                                      m_pixelSamplesHint,
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
        //TODO
        //for (max ray depth)
            //add zeros to geoTerms_L and geoTerms_E
            //add false to isDirac_L and isDirac_E
            //add zeros to PdfPsa_L and PdfPsa_E
            //add (0,0,0)s to position_L and position_E
            //add (0,0,0)s to normal_L and normal_E
            //add (0,0,0)s to outdir_L and outdir_E
            //add NULLs to BSDF_L and BSDF_E
            //add (1,1,1)s to vert_Fs_L and vert_Fs_E
            //add zeros to alpha_i_L and alpha_i_E

            //add two (0,0,0)s to xst_outdir
            //add two zeros to xst_GeoTerm

        // For each pixel row...
        for (size_t y = m_ystart; y < m_yend; ++y)
        {
            // For each pixel across the row...
            for (size_t x = m_xstart; x < m_xend; ++x)
            {
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
                    pixelColor += pathTrace(ray,
                                            m_masterSet,
                                            m_lights,
                                            rng,
                                            samplers,
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
                    samplers.m_lightSelectionSamplers[i]->refill(rng.nextUInt32());
                    samplers.m_lightElementSamplers[i]->refill(rng.nextUInt32());
                    samplers.m_lightSamplers[i]->refill(rng.nextUInt32());
                    samplers.m_bsdfSamplers[i]->refill(rng.nextUInt32());
                    samplers.m_lightBounceSamplers[i]->refill(rng.nextUInt32());
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
            delete samplers.m_lightSelectionSamplers[i];
            delete samplers.m_lightElementSamplers[i];
            delete samplers.m_lightSamplers[i];
            delete samplers.m_bsdfSamplers[i];
            delete samplers.m_lightBounceSamplers[i];
        }
        delete samplers.m_lensSampler;
        delete samplers.m_timeSampler;
        delete samplers.m_subpixelSampler;
        delete samplers.m_BDlightSelectionSampler;
        delete samplers.m_BDlightElementSampler;
        delete samplers.m_BDlightSampler;
        delete samplers.m_BDlightDirectionSampler;
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
        
        // Add in emission when directly visible or via perfect specular bounces
        // (Note that we stop including it through any non-Dirac bounce to
        // prevent caustic noise.)
        if (true || numBounces == 0 || numBounces == numDiracBounces)   //actually, if we want specular caustics, we need to do this for every bounce
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

//bidirectional pathTrace function
Color BDpathTrace(const Ray& ray,
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
    //Calculate We(0) and We(1)
    //TODO
    //Calculate PA(z0)
    //TODO
    //set the first vert_position to the camera's position
    path.m_position_E[0] = ray.m_origin;
    //set the first raydir
    path.m_outdir_E[0] = -ray.m_direction;
    //set the first vert_PDFPSA
    path.m_PdfPsa_E[0] = 1.0f;
    //initialize the eye vertex count, nE, to 1
    int nE = 1;
    //start with the initial ray from the camera
    Ray currentRay = ray;
    size_t numBounces = 0;
    //while depth < max depth
    while(numBounces < samplers.m_maxRayDepth){
        //if numbounces > min depth, do russian roulette
        if(numBounces > samplers.m_minRayDepth){
            //TODO
            //get the maximum component of the eye throughput
            //get a random number
            //if the random number is greater than the max component
                //you've been killed
                //break
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
        Point position = path.m_position_E[nE] = intersection.position();
        //set vertex normal[nE]
        Vector normal = path.m_normal_E[nE] = intersection.m_normal;
        //set outdir_E[nE-1] (remember to negate it here)
        Vector outgoing = path.m_outdir_E[nE] = -currentRay.m_direction;
        Bsdf* pBsdf = NULL;
        float bsdfWeight = 1.0f;
        Color matColor = intersection.m_pMaterial->evaluate(position,
                                                            normal,
                                                            outgoing,
                                                            pBsdf,
                                                            bsdfWeight);
        // No BSDF? We can't evaluate lighting, so bail.
        if(pBsdf == NULL){
            return result;
        }
        //set vertex BSDF[nE]
        path.m_BSDF_E[nE] = pBsdf;
        //set edge GeoTerm[nE-1]
        //TODO
        //set vertex isDirac[nE]
        path.m_isDirac_E[nE] = pBsdf->isDiracDistribution();
        //if this bounce is a light
            //WHAT DO WE DO?
            //we've found a light, so let's not look for another one
            //set foundLight to true
        //Sample the BSDF to find the direction the next leg of the path goes in
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
        //set vertex PdfPsa[nE]
        //calculate Alpha E sub i: (Veach, pg 304)
        //if nE is 0 (t is 1)
            //Alpha E sub i is We^0(z0) / PA(z0)
        //if nE is >= 1 (t >= 2)
            //Alpha E sub i is BSDF(zi-1 -> zi-2 -> zi-3) / PSA(zi-2 -> zi-1) * Alpha E sub i-1
        //update the eye throughput //TODO
        //increment nE
        nE++;
        //increment numBounces
        numBounces++;
    }

    //BUILD A PATH STARTING FROM THE LIGHT
    //initialize the light throughput (for russian roulette) to (1,1,1)
    //Pick a random light and get a sample from it (a position and direction)
    //Calculate Le(0) and Le(1)
    //Calculate PA(y0)
    //set the last vert_position to the light's position
    //set the last vert_PDFPSA
    //initialize the light vertex count, nL, to 1
    //create a ray using the light's position and direction
    //while depth < max depth
        //if numbounces > min depth, do russian roulette
            //get the maximum component of the light throughput
            //get a random number
            //if the random number is greater than the max component
                //you've been killed
                //break
        //Trace the ray to see if we hit anything
        //if we didn't
            //we can still use this light sample, so we're not done with this path
            //break, but don't return
        //store the intersection in vert_intersection????
        //evaluate the material and intersection information at this bounce
        //set lightpath vertex position [nL]
        //set lightpath vertex normal[nL]
        //set lightpath vertex BSDF[nL]
        //set lightpath edge GeoTerm[nL-1]
        //set outdir_L[nL-1]
        //set lightpath vertex isDirac[nL] to false
        //if this BSDF is perfect specular
            //set lightpath vertex isDirac[nL] to true
        //if this bounce is a light
            //WHAT DO WE DO????
            //let's pretend it never happened and end the light path here
            //TODO
        //Sample the BSDF to find the direction the next leg of the path goes in
        //if the BsdfPdf > 0
            //update currentRay with the new direction and origin
        //else
            //we're done here, so break
        //set vertex PdfPsa[nL]
        //calculate Alpha L sub i[nL]: (Veach, pg 304)
        //if nL is 1
            //Alpha L sub i is Le^0(z0) / PA(y0)
        //if nL is 2
            //Alpha L sub i is (Le(1) / PSA[0]) * Alpha L sub i[nL-1]
        //else
            //Alpha L sub i is BSDF(yi-3 -> yi-2 -> yi-1) / PSA(yi-2 -> yi-1) * Alpha L sub i-1
            //Alpha L sub i is (Fs_L[nL-2] / PSA[nL-2]) * Alpha L sub i[nL-1]
        //update the light throughput //TODO
        //increment nL
        //increment numBounces

    //If we already found a light
    //should we pretend it didn't happen?
    //or should we just skip the creation of a light path?

    //BUILD VARIATIONS OF THE COMBINED PATHS
    //for all possible values of S (0 -> nL)
        //for all possible values of T (1 -> nE) (we start at 1 so that we don't create contributions for other pixels)
        //SUM THEIR CONTRIBUTIONS ACCORDING TO THEIR WEIGHTS AND PROBABILITY DISTRIBUTIONS
            //if vertex S or vertex T is a Dirac distribution
                //skip this path
            //calculate the Pdf for connecting the vertex T to vertex S
            //if it's 0
                //skip this path
            //calculate the geometric term for the connecting edge
            //if it's 0
                //skip this path
            // Create a combined path, X_st
            //outgoing ray directions
            //geometric terms
            // Calculate the MIS weight for this path, Wst (Veach, pg 306)
            //set p_sum to 1 (we're adding ps^2 to start)
            //set p_cur to 1
            //set p_next to 0
            //set p_pre to 0
            // Get the P_i values greater than P_s
            //for idx from S+1 to S+T-1:
                //p_next = (pdfPSA(x_idx-1 -> x_idx) * GeoTerm(x_idx-1 <-> x_idx)) / (pdfPSA(x_idx+1 -> x_idx) * GeoTerm(x_idx+1 <-> x_idx)) * p_cur
                //p_sum += p_next^2
                //p_cur = p_next
            // Get P_k+1
            //p_sum += ((pdfPSA(x_k-1 -> x_k) * GeoTerm(x_k-1 <-> x_k)) / (P_A(x_k))) ^ 2
            // Get the P_i values smaller than P_s
            //set p_cur to 1
            //for idx from S-1 to 0:
                //p_pre = (pdfPSA(x_idx+1 -> x_idx) * GeoTerm(x_idx+1 <-> x_idx)) / (pdfPSA(x_idx-1 -> x_idx) * GeoTerm(x_idx-1 <-> x_idx)) * p_cur
                //p_sum += p_pre^2
                //p_cur = p_pre
            // Get P_0
            //p_sum += ((pdfPSA(x_1 -> x_0) * GeoTerm(x_1 <-> x_0)) / (P_A(x_0))) ^ 2
            //Wst = 1 / p_sum

            //calculate Cst (Veach, pg 305)
            //if s == 0
                //Cst = Le(z_t-1 -> z_t-2)
            //else if t == 0 (which it won't be)
                //Cst = We(y_s-2 -> y_s-1)
            //else
                //Cst = BSDF(y_s-2 -> y_s-1 -> z_t-1) * G(y_s-1 <-> z_t-1) * BSDF(y_s-1 -> z_t-1 -> z_t-2)
            //add Wst * Cst * Alpha_L_S * Alpha_E_T to the result

    //return the result
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
