#include "ObjectReader.h"
#include "middleware/GenomPoster.h"
#include "vimanStruct.h"

class VimanObjectReader : public ObjectReader
{

  public:
    //Constructor
    VimanObjectReader();
    //Destructor
    ~VimanObjectReader();
    
    void init();
    void updateObjects();
    
  private:
    GenomPoster* vimanPoster_;
    VimanObjectPublicArray vimanPosterStruct_;    
};
