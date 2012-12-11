/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Antonio El Khoury (LAAS-CNRS)
*/

#ifndef KWS_PLUS_BENCH_ROADMAP_BUILDER
# define KWS_PLUS_BENCH_ROADMAP_BUILDER

# include <vector>
# include <cmath>

# include <boost/accumulators/accumulators.hpp>
# include <boost/accumulators/statistics/stats.hpp>
# include <boost/accumulators/statistics/mean.hpp>
# include <boost/accumulators/statistics/variance.hpp>
# include <boost/accumulators/statistics/min.hpp>
# include <boost/accumulators/statistics/max.hpp>

# include <KineoUtility/kitInterface.h>
# include <KineoWorks2/kwsDiffusingRdmBuilder.h>
# include <KineoWorks2/kwsDefine.h>

KIT_PREDEF_CLASS( CkwsRoadmap );

template<class T = CkwsDiffusingRdmBuilder > class CkwsPlusBenchRdmBuilder; 

class CkwsDiffusingRdmBuilder;

class Timer
{
public:

  Timer() : m_start(), m_stop(), t(0) {};
    
  void start() { ::gettimeofday(&m_start, NULL); }
    
  void stop()
  {
    ::gettimeofday(&m_stop, NULL);
    t += ( m_stop.tv_sec - m_start.tv_sec ) * 1e6
      + ( m_stop.tv_usec - m_start.tv_usec );
  }
    
  void reinit() { t = 0; }

  /// \brief Get time in milliseconds.
  double get() { return (double)t / 1e3; }
    
private:
  struct timeval m_start;
  struct timeval m_stop;
  long t;
};

/// \addtogroup Bench
/// \{
/// This part implements a roadmap builder used for benchmarking
/// purposes.

/// \brief This template class inherits from any diffusing roadmap
/// builder class, and logs benchmark data.
template<class T >
class CkwsPlusBenchRdmBuilder : public T
{
 public:
  
  typedef boost::accumulators::accumulator_set
  <double,
   boost::accumulators::features
   <boost::accumulators::tag::mean,
    boost::accumulators::tag::lazy_variance,
    boost::accumulators::tag::min,
    boost::accumulators::tag::max> >
  accumulator_t;

  /// \brief Destructor.
  ~CkwsPlusBenchRdmBuilder();

  /// \brief Constructor.
  /// \param i_roadmap The roadmap to construct.
  /// \param i_penetration The penetration allowed.
  /// \param i_evaluator The class used to evaluate the distance
  /// between two configurations.
  /// \param i_picker The picker used to pick diffusion nodes.
  /// \param i_shooter The shooter used to generate the configurations.
  static 
    KIT_SHARED_PTR(CkwsPlusBenchRdmBuilder<T>)
    create (const CkwsRoadmapShPtr& i_roadmap,
	    double i_penetration,
	    const CkwsDistanceShPtr& i_evaluator = CkwsDistance::create (),
	    const CkwsDiffusionNodePickerShPtr &i_picker
	    = CkwsPickerBasic::create (), 
	    const CkwsDiffusionShooterShPtr &i_shooter
	    = CkwsShooterRoadmapBox::create ());

  /// \brief Copy constructor.
  /// \param inRdmpBuilder The roadmap builder to copy.
  static 
    KIT_SHARED_PTR(CkwsPlusBenchRdmBuilder<T>)
    createCopy (const KIT_SHARED_PTR_CONST (CkwsPlusBenchRdmBuilder<T>)
		&inRdmpBuilder);

  unsigned int getStepsCount ();

  double getStepsTotalTime ();

  double getStepsMeanTime ();

  double getStepsVarianceTime ();

  double getStepsMinTime ();

  double getStepsMaxTime ();

  unsigned int getDiffusionsCount ();

  double getDiffusionsTotalTime ();

  double getDiffusionsMeanTime ();

  double getDiffusionsVarianceTime ();

  double getDiffusionsMinTime ();

  double getDiffusionsMaxTime ();

  unsigned int getExtensionsCount ();

  double getExtensionsTotalTime ();

  double getExtensionsMeanTime ();

  double getExtensionsVarianceTime ();

  double getExtensionsMinTime ();

  double getExtensionsMaxTime ();

  unsigned int getNodeCount ();

  void resetAccumulators ();

 protected:
  /// \brief Constructor
  /// \param i_roadmap: The roadmap to be built,
  CkwsPlusBenchRdmBuilder (const CkwsRoadmapShPtr& i_roadmap);

  /// \brief Init function
  /// \param i_weakPtr The weak pointer to the BenchRdmBuilder.
  /// \return ktStatus KD_OK or KD_ERROR
  ktStatus init (const KIT_WEAK_PTR(CkwsPlusBenchRdmBuilder<T>)& i_weakPtr);

  ktStatus buildOneStep ();

  CkwsNodeShPtr diffuse (const CkwsNodeShPtr& i_node,
  			 CkwsDiffusingRdmBuilder::EDiffusionNodeType i_type,
  			 CkwsRoadmapBuilder::EDirection& o_direction);

  CkwsNodeShPtr extend (const CkwsNodeShPtr & i_node,
  			const CkwsConfig & i_cfg,
  			CkwsRoadmapBuilder::EDirection i_direction);

 private:
  KIT_WEAK_PTR(CkwsPlusBenchRdmBuilder<T>) m_weakPtr;

  Timer timer_;

  accumulator_t accSteps_;
  accumulator_t accDiffusions_;
  accumulator_t accExtensions_;

};

template<class T>
CkwsPlusBenchRdmBuilder<T>::~CkwsPlusBenchRdmBuilder ()
{
}

template<class T>
KIT_SHARED_PTR(CkwsPlusBenchRdmBuilder<T>)
CkwsPlusBenchRdmBuilder<T>::
create (const CkwsRoadmapShPtr &i_roadmap,
	double i_penetration,
	const CkwsDistanceShPtr &i_evaluator,
	const CkwsDiffusionNodePickerShPtr &i_picker,
	const CkwsDiffusionShooterShPtr &i_shooter)
{
  CkwsPlusBenchRdmBuilder<T> * rdmBuilderPtr
    = new CkwsPlusBenchRdmBuilder<T>(i_roadmap);
  KIT_SHARED_PTR(CkwsPlusBenchRdmBuilder<T>) rdmBuilderShPtr (rdmBuilderPtr);
  KIT_WEAK_PTR(CkwsPlusBenchRdmBuilder<T>) rdmBuilderWkPtr (rdmBuilderShPtr);

  if (rdmBuilderPtr->init (rdmBuilderWkPtr) != KD_OK){
    rdmBuilderShPtr.reset ();
  }
  else {
    rdmBuilderPtr->distance (i_evaluator);
    rdmBuilderPtr->diffusionNodePicker (i_picker);
    rdmBuilderPtr->diffusionShooter (i_shooter);
    if(KD_ERROR == CkwsValidatorDPCollision::setPenetration
       (rdmBuilderShPtr->builderDirectPathValidator (), i_penetration))
      rdmBuilderShPtr.reset();
  }

  return rdmBuilderShPtr;
}

template<class T>
KIT_SHARED_PTR(CkwsPlusBenchRdmBuilder<T>)
CkwsPlusBenchRdmBuilder<T>::
createCopy (const KIT_SHARED_PTR_CONST(CkwsPlusBenchRdmBuilder<T>)
	    &inRdmpBuilder)
{
  if (inRdmpBuilder != NULL) {
    CkwsPlusBenchRdmBuilder<T>* rdmBuilderPtr
      = new CkwsPlusBenchRdmBuilder<T> (*inRdmpBuilder);
    KIT_SHARED_PTR(CkwsPlusBenchRdmBuilder<T>)
      rdmBuilderShPtr (rdmBuilderPtr);
    KIT_WEAK_PTR(CkwsPlusBenchRdmBuilder<T>)
      rdmBuilderWkPtr (rdmBuilderShPtr);

    if(rdmBuilderPtr->init (rdmBuilderWkPtr) != KD_OK)
      rdmBuilderShPtr.reset ();
    else
      {
	rdmBuilderPtr->distance(inRdmpBuilder->distance());
	double penetration;
	if (KD_ERROR == CkwsValidatorDPCollision::getPenetration
	    (inRdmpBuilder->builderDirectPathValidator (), penetration))
	  rdmBuilderShPtr.reset ();
	else
	  if(KD_ERROR == CkwsValidatorDPCollision::setPenetration
	     (rdmBuilderShPtr->builderDirectPathValidator (), penetration))
	    rdmBuilderShPtr.reset ();
      }
    return rdmBuilderShPtr;
  }
  return KIT_SHARED_PTR(CkwsPlusBenchRdmBuilder<T>) ();
}

template<class T>
CkwsPlusBenchRdmBuilder<T>::CkwsPlusBenchRdmBuilder
(const CkwsRoadmapShPtr& i_roadmap)
: T(i_roadmap),
  timer_ (),
  accSteps_ (),
  accDiffusions_ (),
  accExtensions_ ()
{
}

template<class T>
ktStatus CkwsPlusBenchRdmBuilder<T>::init
(const KIT_WEAK_PTR(CkwsPlusBenchRdmBuilder<T>)& i_weakPtr)
{
  ktStatus success = T::init (i_weakPtr);

  if (KD_OK == success){
    m_weakPtr = i_weakPtr;
  }
  return success ;
}

template<class T>
unsigned int CkwsPlusBenchRdmBuilder<T>::
getStepsCount ()
{
  return boost::accumulators::count (accSteps_);
}

template<class T>
double CkwsPlusBenchRdmBuilder<T>::
getStepsTotalTime ()
{
  return boost::accumulators::sum (accSteps_) / 1e3;
}

template<class T>
double CkwsPlusBenchRdmBuilder<T>::
getStepsMeanTime ()
{
  return boost::accumulators::mean (accSteps_) / 1e3;
}

template<class T>
double CkwsPlusBenchRdmBuilder<T>::
getStepsVarianceTime ()
{
  return boost::accumulators::variance (accSteps_) / 1e6;
}

template<class T>
double CkwsPlusBenchRdmBuilder<T>::
getStepsMinTime ()
{
  return boost::accumulators::min (accSteps_) / 1e3;
}

template<class T>
double CkwsPlusBenchRdmBuilder<T>::
getStepsMaxTime ()
{
  return boost::accumulators::max (accSteps_) / 1e3;
}

template<class T>
unsigned int CkwsPlusBenchRdmBuilder<T>::
getDiffusionsCount ()
{
  return boost::accumulators::count (accDiffusions_);
}

template<class T>
double CkwsPlusBenchRdmBuilder<T>::
getDiffusionsTotalTime ()
{
  return boost::accumulators::sum (accDiffusions_) / 1e3;
}

template<class T>
double CkwsPlusBenchRdmBuilder<T>::
getDiffusionsMeanTime ()
{
  return boost::accumulators::mean (accDiffusions_) / 1e3;
}

template<class T>
double CkwsPlusBenchRdmBuilder<T>::
getDiffusionsVarianceTime ()
{
  return boost::accumulators::variance (accDiffusions_) / 1e6;
}

template<class T>
double CkwsPlusBenchRdmBuilder<T>::
getDiffusionsMinTime ()
{
  return boost::accumulators::min (accDiffusions_) / 1e3;
}

template<class T>
double CkwsPlusBenchRdmBuilder<T>::
getDiffusionsMaxTime ()
{
  return boost::accumulators::max (accDiffusions_) / 1e3;
}

template<class T>
unsigned int CkwsPlusBenchRdmBuilder<T>::
getExtensionsCount ()
{
  return boost::accumulators::count (accExtensions_);
}

template<class T>
double CkwsPlusBenchRdmBuilder<T>::
getExtensionsTotalTime ()
{
  return boost::accumulators::sum (accExtensions_) / 1e3;
}

template<class T>
double CkwsPlusBenchRdmBuilder<T>::
getExtensionsMeanTime ()
{
  return boost::accumulators::mean (accExtensions_) / 1e3;
}

template<class T>
double CkwsPlusBenchRdmBuilder<T>::
getExtensionsVarianceTime ()
{
  return boost::accumulators::variance (accExtensions_) / 1e6;
}

template<class T>
double CkwsPlusBenchRdmBuilder<T>::
getExtensionsMinTime ()
{
  return boost::accumulators::min (accExtensions_) / 1e3;
}

template<class T>
double CkwsPlusBenchRdmBuilder<T>::
getExtensionsMaxTime ()
{
  return boost::accumulators::max (accExtensions_) / 1e3;
}

template<class T>
unsigned int CkwsPlusBenchRdmBuilder<T>::
getNodeCount ()
{
  return T::roadmap ()->countNodes ();
}

template<class T>
ktStatus CkwsPlusBenchRdmBuilder<T>::
buildOneStep ()
{
  timer_.reinit ();
  timer_.start ();
  ktStatus success = T::buildOneStep ();
  timer_.stop ();

  accSteps_ (timer_.get ());

  if (boost::accumulators::count (accSteps_) > 20000)
    return KD_ERROR;
  else
    return success;
}

template<class T>
void CkwsPlusBenchRdmBuilder<T>::
resetAccumulators ()
{
  accSteps_ = accumulator_t ();
  accDiffusions_ = accumulator_t ();
  accExtensions_ = accumulator_t ();
}

template<class T>
CkwsNodeShPtr CkwsPlusBenchRdmBuilder<T>::
diffuse (const CkwsNodeShPtr& i_node,
	 CkwsDiffusingRdmBuilder::EDiffusionNodeType i_type,
	 CkwsRoadmapBuilder::EDirection& o_direction)
{
  timer_.reinit ();
  timer_.start ();
  CkwsNodeShPtr node = T::diffuse (i_node, i_type, o_direction);
  timer_.stop ();

  accDiffusions_ (timer_.get ());

  return node;
}

template<class T>
CkwsNodeShPtr CkwsPlusBenchRdmBuilder<T>::
extend(const CkwsNodeShPtr & i_node,
       const CkwsConfig & i_cfg,
       CkwsRoadmapBuilder::EDirection i_direction )
{
  timer_.reinit ();
  timer_.start ();
  CkwsNodeShPtr node = T::extend (i_node, i_cfg, i_direction);
  timer_.stop ();

  accExtensions_ (timer_.get ());

  return node;
}

typedef CkwsPlusBenchRdmBuilder<CkwsDiffusingRdmBuilder>
CkwsPlusBenchDiffusingRdmBuilder;
KIT_POINTER_DEFS(CkwsPlusBenchDiffusingRdmBuilder);

/// \}

#endif
