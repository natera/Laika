/*
 * BALINESOFT PRESENTS:
 *
 * "Laika come home"
 *                           ;\
                            |' \
         _                  ; : ;
        / `-.              /: : |
       |  ,-.`-.          ,': : |
       \  :  `. `.       ,'-. : |
        \ ;    ;  `-.__,'    `-.|
         \ ;   ;  :::  ,::'`:.  `.
          \ `-. :  `    :.    `.  \
           \   \    ,   ;   ,:    (\
            \   :., :.    ,'o)): ` `-.
           ,/,' ;' ,::"'`.`---'   `.  `-._
         ,/  :  ; '"      `;'          ,--`.
        ;/   :; ;             ,:'     (   ,:)
          ,.,:.    ; ,:.,  ,-._ `.     \""'/
          '::'     `:'`  ,'(  \`._____.-'"'
             ;,   ;  `.  `. `._`-.  \\
             ;:.  ;:       `-._`-.\  \`.
              '`:. :        |' `. `\  ) \
                 ` ;:       |    `--\__,'
                   '`      ,'
                        ,-'
                        *
 * "A brief story of drones and autonomous landing"
 *
 * Basic structure of the program:
 *
 * 1. Start video stream
 * 2. Change colorspace to HSV
 * 3. Apply color filter
 * 4. Select ROI (RotatedRec and Bounding Rectangle)
 * 5. Find center circle area
 * 6. Get distance from area
 * 7. Center drone
 * 8. Send control command!
 *
 * */
#ifndef VISION_H_
#define VISION_H_


int vision_main_thread();//( int argc, char** argv );

#endif // VISION_H_
