#ifndef CELL_H
#define CELL_H

/*!
 *  \brief     Cell Class
 *  \details
 *  This class is used to describe cells that are used in fusion.\n
 *  \author    Alen Alempijevic
 *  \version   1.01-2
 *  \date      2019-07-10
 *  \pre       none
 *  \bug       none reported as of 2020-04-11
 *  \warning   students MUST NOT change this class (the header or implementation file)
 */


namespace pfms {

  //http://wiki.ros.org/CppStyleGuide#Enumerations
  namespace cell {

  typedef enum {
    UNKNOWN=0,
    FREE=1,
    OCCUPIED=-1
  } State; /*!< Available cell states*/

  }

  /**
  @class Cell
  @brief Cells will be used as areas of space (rectangles) where the sensor data will be fused to indicate occupancy, default size provided, centre location draw randonly from a map of max size. On creation state is UNKNOWN
  @details The Cell will be used for fusion accordinly: \n
  If the sensor intersects the cell and goes through Cell will be FREE\n
  If the sensor has a return from within Cell, it will be OCCUPIED\n*/
  class Cell {

  // Public members are accessible from outside the class (ie. in main)
  public:
      /**
      Constructor with parameters to set the cell centre and side length
      @param centreX X coordinate of the centre of the cell [m]
      @param centreY Y coordinate of the centre of the cell [m]
      @param side Side length of the cell [m]
      @note The cell is square, so the side length is the same for both width and height, centred at the given coordinates and the state is set to UNKNOWN
      */
      Cell(double centreX, double centreY, double side);

      /**
      Constructor with parameters to set the cell centre and side length
      @param centreX X coordinate of the centre of the cell [m]
      @param centreY Y coordinate of the centre of the cell [m]
      @param side Side length of the cell [m]
      @param seq Sequence number of the cell, used for tracking changes in rviz
      @note The cell is square, so the side length is the same for both width and height, centred at the given coordinates and the state is set to UNKNOWN
      */
      Cell(double centreX, double centreY, double side, unsigned int seq);


      /**
      Member function sets Side
      @param side The desired side length
      */
      void setSide (double side);

      /**
      Member function to get value of the side of cell
      @return side of cell [m]
      */
      double getSide (void);

      /**
      Member function to get area of cell
      @return area of cell  [m2]
      */
      double area(void);

      /**
      Member function to get perimeter of cell
      @return perimieter of cell [m]
      */
      double perimeter(void);

      /**
      Member function to set centre of cell
      @param x centre coordinate x [m]
      @param y centre coordinate y [m]
      */
      void setCentre(double x, double y);
      /**
      Member function to get centre of cell
      @param x centre coordinate x [m]
      @param y centre coordinate y [m]
      */
      void getCentre(double &x, double &y);

      /**
      Member function to get state of cell
      @return state of cell
      */
      cell::State getState();

      /**
      Member function to set state of cell
      @param area of cell
      */
      void setState(cell::State);

      /**
       * Member function to get sequence number of the cell
       * @return Sequence number of the cell, used for tracking changes in rviz
       */
      unsigned int getSeq(void) { return seq_ ;}

      /**
       * Member function to check if sequence number has been set manually (used in constructor with parameters)
       * @return true if sequence number has been set manually, false otherwise
       */
      bool isSeqSet(void) { return isSeqSet_; }

  // Private members are only accessible from within methods of the same class
  private:

      /*! @brief Sets the sequence number of the cell to a random value
      @details The sequence number is set to a random value using a random number generator seeded with the current time. 
      This is used to track changes in the cell's state for visualisation purposes in RViz.
      */
      void setSeq(void);

      double side_;   //!< Sides of the cell (which is rectangle shape so has equal width and height)
      double centreX_;//!< X coordinate of centre of shape
      double centreY_;//!< Y coordinate of centre of shape
      cell::State state_;   //!< State of cell
      unsigned int seq_ ; //!< Sequence number for the cell, used for tracking changes
      bool isSeqSet_; //!< Flag to indicate if the sequence number has been set manually (used in constructor with parameters)
  };

} // namespace pfms
#endif // CELL_H
