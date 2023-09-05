#include "yarp/os/BufferedPort.h"
#include "yarp/os/Network.h"
#include "yarp/sig/Vector.h"
#include "yarp/os/ConnectionWriter.h"
#include "yarp/os/Portable.h"

#include "FootstepsViewerRos.hpp"
#include <thread>

class FootstepsViewerYarp : public yarp::os::PortReader
{
private:
    std::mutex m_mutex;
    std::shared_ptr<FootstepsViewerRos> m_rosNode;
    bool m_initialized = false;

public:
    FootstepsViewerYarp();
    //main loop executed for each port reading of the merged feet status
    bool read(yarp::os::ConnectionReader& t_connection) override;

    void runROS();
};  //End of class FootstepsViewerYarp : public yarp::os::PortReader
