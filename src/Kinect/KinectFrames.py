from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

import ctypes
import _ctypes
import pygame
import sys



class frameGrab(object):
    def __init__(self):
        pygame.init()

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Set the width and height of the screen [width, height]
        self._infoObject = pygame.display.Info()
        self._screen = pygame.display.set_mode((self._infoObject.current_w >> 1, int((self._infoObject.current_h >> 1)/2)), 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)
        
        pygame.display.set_caption("Kinect Depth Test")

        # Loop until the user clicks the close button.
        self._done = False

        # Kinect runtime object, we want only color and body frames 
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_Color)
    
        # back buffer surface for getting Kinect color frames, 32bit color, width and height equal to the Kinect color frame size
        self.color_frame_surface = pygame.Surface((self._kinect.color_frame_desc.Width, self._kinect.color_frame_desc.Height), 0, 32)
        self.depth_frame_surface = pygame.Surface((self._kinect.depth_frame_desc.Width, self._kinect.depth_frame_desc.Height), 0, 16)
   
    def draw_depth_frame(self, frame, target_surface):
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()

    def draw_color_frame(self, frame, target_surface):
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()

    def run(self):
        # -------- Main Program Loop -----------
        while not self._done:
            # --- Main event loop
            for event in pygame.event.get(): # User did something
                if event.type == pygame.QUIT: # If user clicked close
                    self._done = True # Flag that we are done so we exit this loop

                elif event.type == pygame.VIDEORESIZE: # window resized
                    self._screen = pygame.display.set_mode(event.dict['size'], 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)
                    
            # --- Game logic should go here

            # --- Getting frames and drawing  
            # --- Woohoo! We've got a color frame! Let's fill out back buffer surface with frame's data 
            if self._kinect.has_new_color_frame():
                frame = self._kinect.get_last_color_frame()
                self.draw_color_frame(frame, self.color_frame_surface)
                frame = None           
           
            if self._kinect.has_new_depth_frame():
                frame = self._kinect.get_last_depth_frame()
                self.draw_depth_frame(frame, self.depth_frame_surface)
                frame = None

            
            # --- copy back buffer surface pixels to the screen, resize it if needed and keep aspect ratio
            # --- (screen size may be different from Kinect's color frame size) 
            h_to_w_color = float(self.color_frame_surface.get_height()) / self.color_frame_surface.get_width()
            target_height_color = int(h_to_w_color * (self._screen.get_width()/2))
            surface_to_draw_color = pygame.transform.scale(self.color_frame_surface, (int(self._screen.get_width()/2), target_height_color));

            h_to_w_depth = float(self.depth_frame_surface.get_height()) / self.depth_frame_surface.get_width()
            target_height_depth = int(h_to_w_depth * (self._screen.get_width()/2))
            surface_to_draw_depth = pygame.transform.scale(self.depth_frame_surface, (int(self._screen.get_width()/2), target_height_depth));

            #surface_to_draw = pygame.transform.scale(self.color_frame_surface, (self._screen.get_width(), target_height));
            self._screen.blit(surface_to_draw_color, (int(self._screen.get_width()/2),0))
            self._screen.blit(surface_to_draw_depth, (0,0))
            surface_to_draw_color = None
            surface_to_draw_depth = None
            pygame.display.update()

            # --- Go ahead and update the screen with what we've drawn.
            pygame.display.flip()

            # --- Limit to 60 frames per second
            self._clock.tick(60)

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()
        pygame.quit()


__main__ = "Kinect Depth Test"
game = frameGrab();
game.run();





