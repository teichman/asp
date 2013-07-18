#ifndef TWIDDLER_H
#define TWIDDLER_H

#define BOOST_FILESYSTEM_VERSION 2

#include <signal.h>
#include <boost/filesystem.hpp>
#include <ros/assert.h>
#include <ros/console.h>
#include <bag_of_tricks/bag_of_tricks.h>
#include <bag_of_tricks/high_res_timer.h>
#include <pipeline/params.h>

namespace pl
{

  //! Twiddles a Params object and saves output.  The output is saved in a Dictionary<string, double>
  //! so that you can save whatever you want.  For example, you might include "accuracy" and "timing"
  //! output, then optimize accuracy subject to timing constraints.
  //! See method comments for how exactly to do this.
  //! The objective function value is being minimized.
  class Twiddler
  {
  public:
    //! User-defined results of evaluating a Params set.
    typedef Dictionary<std::string, double> Results;
    std::map<Params, Results> results_;
    
    Twiddler();
    virtual ~Twiddler() {}

    //! Creates a directory and seeds the results with the initial params.
    //! Can call twiddle after this.
    //! rootpath is where the output of each evaluation will be saved.
    //! It must not exist and will be created.
    void initialize(const Params& init, const std::string& rootpath);
    //! Loads the data from a previous run.
    void load(std::string rootpath);
    //! load or initialize must have been called.
    //! generateParamVariation, evaluate, save.
    void twiddle(double max_hours = 0);
    
    /************************************************************
     * Functions to be implemented
     ************************************************************/

    //! You can place additional output in evalpath, if needed.
    //! params can also be found in evalpath + "/params.txt" if you are using an external script.
    virtual Results evaluate(const Params& params, std::string evalpath) = 0;
    //! Given the best current params, generate a new set of params.
    //! You can implement whatever search strategy you want here.
    //! Twiddler will check whether your new Params is a duplicate.
    virtual Params generateParamVariation(Params params) const = 0;
    //! By default, this returns the field called "objective".
    //! You can overload this to handle constraints as well as different objectives.
    //! Lower is better.
    virtual double objective(const Results& results) const;
    //! This is called any time an improved set of params is found.
    virtual void improvementHook(const Params& params, const Results& results, std::string evalpath) const;
    //! Termination criteria.  Never terminates by default.
    virtual bool done(const Results& results) const;

    
    /************************************************************
     * Miscellaneous things.
     ************************************************************/

    void getBest(Params* best_params, Results* best_results) const;
    //! ordered_params->at(0) is the best set of params so far.  Ascending order.
    void getOrdering(std::vector<Params>* params, std::vector<Results>* results, std::vector<double>* objectives) const;

    
    /************************************************************
     * Useful functions for generating parameter variations
     ************************************************************/
    
    static Params randomMerge(const Params& params0, const Params& params1);
    
  protected:
    std::string rootpath_;
  };

}

#endif // TWIDDLER_H
