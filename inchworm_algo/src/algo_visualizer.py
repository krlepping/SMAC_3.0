#!/usr/bin/env python3

import rospy, sys, std_msgs, pygame, math

from inchworm_algo.msg import ShingleMsg, RoofState, InchwormsMsg

from tile_renderer import TileRenderer

roof_state = []
shingle_depots = []
inchworms = []
tile_renderer = None


WIDTH = 1800
HEIGHT = 1050
FPS = 60
## initialize pygame and create window
pygame.init()
pygame.mixer.init()  ## For sound
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Roof Rendering")
clock = pygame.time.Clock()   



def handle_new_inchworms(msg, inchworms):
    inchworms = list(msg.inchworms)



def handle_new_roof_state(msg, screen, roof_state, shingle_depot_pos, inchworms):
    # rospy.loginfo("handling new RoofState")
    roof_state = list(msg.shingles)
    # rospy.loginfo(len(msg.shingles))
    inchworms = list(msg.inchworms)
    shingle_depot_pos = list(msg.depot_positions)
    inchworm_occ = list(msg.inchworm_occ)
    #3 Draw/render
    screen.fill(WHITE)

    tile_renderer.drawRoof(screen, roof_state, shingle_depot_pos, inchworms, inchworm_occ)
    screen.blit(pygame.transform.flip(screen, False, True), (0, 0))

    ## Done after drawing everything to the screen
    pygame.display.flip()
    # rospy.loginfo(pygame.display.Info())




if __name__ == '__main__':
    # TODO: pass in a param to allow you to viz the map of a given inchworm
    rospy.init_node("algo_visualizer")


    inchworms = []



    # Define Colors
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)

  ## For syncing the FPS

    ### Rendering constants ###
    TILE_BUFFER = 1 # px between drawn tiles
    ###########################

    ### Project constants ###
    ROOF_WIDTH  = int(sys.argv[1]) # tiles
    ROOF_HEIGHT = int(sys.argv[2]) # tiles


    #########################

    MARGINS = 50

    tile_renderer = TileRenderer(WIDTH , HEIGHT , ROOF_HEIGHT, ROOF_WIDTH, TILE_BUFFER, MARGINS)

    roof_state = [1] * WIDTH * HEIGHT
    shingle_depot_pos = [0, 5]
    


    rospy.Subscriber('algo/roof_state', RoofState, lambda x: handle_new_roof_state(x, screen, roof_state, shingle_depot_pos, inchworms))
    rospy.Subscriber('algo/inchworms', InchwormsMsg, lambda x: handle_new_inchworms(x, inchworms))
    rospy.loginfo(f" roof size {len(roof_state)}")
    running = True
    while not rospy.is_shutdown() and running:
        #1 Process input/events
        clock.tick(FPS)     ## will make the loop run at the same speed all the time
        for event in pygame.event.get():        # gets all the events which have occured till now and keeps tab of them.
            ## listening for the the X button at the top
            if event.type == pygame.QUIT:
                running = False

        
    pygame.quit()
