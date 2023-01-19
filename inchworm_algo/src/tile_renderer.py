#!/usr/bin/env python

import math, pygame, rospy





class TileRenderer:

    # colors
    NO_TILE = (50, 50, 50)

    PLACED_TILE = (100, 100, 255)

    FRONTIER_TILE = (100, 0, 100)
    INSTALLED_TILE = (0, 0, 200)

    INCHWORM_LINKS = (100, 100, 100)
    INCHWORM_EE_PLACED = (0, 255, 0)
    INCHWORM_EE_IN_AIR = (200, 0, 0)

    DEPOT_COLOR = (0, 200, 200)
    OCCUPIED_TILE = (255, 0, 0)
    VALID_MOVE = (0, 255, 0)



    def __init__(self, screen_width, screen_height, roof_height, roof_width, tile_buffer, roof_margin):
        self.tile_buffer = tile_buffer

        self.roof_margin = roof_margin
        self.screen_width = screen_width
        
        self.num_tiles_wide = roof_width
        self.num_tiles_high = roof_height
        rospy.loginfo(f"desired roof width in tiles {self.num_tiles_wide}")
        rospy.loginfo(f"desired roof height in tiles {self.num_tiles_high}")

        self.tile_width_px = int((screen_width - 2 *roof_margin) / (self.num_tiles_wide - .5)) - self.tile_buffer
        self.tile_height_px = int(screen_height / self.num_tiles_high) - self.tile_buffer

        rospy.loginfo(f"Number of tiles wide: {self.num_tiles_wide}")
        rospy.loginfo(f"Number of tiles high: {self.num_tiles_high}")

        rospy.loginfo(f"Tile width px: {self.tile_width_px}")
        rospy.loginfo(f"Tile height px: {self.tile_height_px}")

    def getTileRect(self, x, y):
        #       x0 y0 w  h
        rect = [0, 0, 0, self.tile_height_px]

        rect[0] = x * (self.tile_width_px) + self.roof_margin
        rect[1] = y * (self.tile_height_px + self.tile_buffer)

        # If we are on an odd row, offset x
        if y % 2 == 1:
            if x == 0:
                rect[2] = int(self.tile_width_px / 2)
                rect[0] = max(rect[0] - int(self.tile_width_px / 2), self.roof_margin)
            elif x == self.num_tiles_wide:
                rect[2] = int(self.tile_width_px / 2)
                rect[0] = max(rect[0] - int(self.tile_width_px / 2), 0) + x * self.tile_buffer
                
            else:
                rect[2] = self.tile_width_px

                rect[0] = max(rect[0] - int(self.tile_width_px / 2), 0) + x * self.tile_buffer
        else:
            if x == self.num_tiles_wide - 1:
                rect[2] = int(self.tile_width_px / 2) + self.tile_buffer
                rect[0] += x * self.tile_buffer
            elif x != 0:
                rect[2] = self.tile_width_px
                rect[0] += x * self.tile_buffer

            else:
                rect[2] = self.tile_width_px
                

        return rect

    def draw_inchworm(self, screen, id, ee_bot_pos, ee_bot_status, ee_top_pos, ee_top_status):
        # make this draw a line instead with some way to know id and top vs bot
        text_size = min(self.tile_height_px, self.tile_width_px)
        font = pygame.font.SysFont("arial", int(text_size/4))
        ee_bot_x = ee_bot_pos[0] * (self.tile_width_px + 1) + self.tile_width_px/2 + self.roof_margin
        ee_bot_y = ee_bot_pos[1] * (self.tile_height_px + 1) + self.tile_height_px/2
        if ee_bot_pos[1] % 2 == 1:
             ee_bot_x -= self.tile_width_px/2
        ee_top_x = ee_top_pos[0] * (self.tile_width_px + 1) + self.tile_width_px/2 + self.roof_margin
        ee_top_y = ee_top_pos[1] * (self.tile_height_px + 1) + self.tile_height_px/2
        if ee_top_pos[1] % 2 == 1:
             ee_top_x -= self.tile_width_px/2
        pygame.draw.line(screen, self.INCHWORM_LINKS, (ee_bot_x, ee_bot_y), (ee_top_x, ee_top_y), int(text_size/8))

        color = self.INCHWORM_EE_PLACED
        if ee_bot_status == 1:
            color = self.INCHWORM_EE_IN_AIR
        pygame.draw.circle(screen, color, (int(ee_bot_x), int(ee_bot_y)), int(text_size/4))

        text = font.render((str(id)), True, (0, 0, 0))
        text = pygame.transform.flip(text, False, True)
        text_rect = text.get_rect()
        text_rect.centerx = ee_bot_x
        text_rect.centery = ee_bot_y
        screen.blit(text, text_rect)

        color = self.INCHWORM_EE_PLACED
        if ee_top_status == 1:
            color = self.INCHWORM_EE_IN_AIR
        pygame.draw.circle(screen, color, (int(ee_top_x), int(ee_top_y)), int(text_size/4))




    def drawRoof(self, screen, roof_state, shingle_depots_pos, inchworms, inchworm_occ):
        # draw shingle states
        # rospy.loginfo(len(roof_state))
        for row in range(self.num_tiles_high):
            for col in range(self.num_tiles_wide):
                rect = self.getTileRect(col, row)
                color = self.NO_TILE
                # rospy.loginfo(row * self.num_tiles_wide + col)
                if roof_state[row * self.num_tiles_wide + col] == 1:
                    color = self.PLACED_TILE
                elif roof_state[row * self.num_tiles_wide + col] == 2:
                    color = self.INSTALLED_TILE
                elif roof_state[row * self.num_tiles_wide + col] == 3:
                    color = self.FRONTIER_TILE # not  implemented yet, not sure if we want the occupany grid to track this
                pygame.draw.rect(screen, color, rect)

        for inchworm in inchworms:
            for index in range(0, len(inchworm.bottom_foot_valid_neighbors), 2):
                rect = self.getTileRect(inchworm.bottom_foot_valid_neighbors[index], inchworm.bottom_foot_valid_neighbors[index + 1])
                color = self.VALID_MOVE
                if (inchworm.bottom_foot_valid_neighbors[index] > -1 and inchworm.bottom_foot_valid_neighbors[index + 1] > -1 and
                        inchworm.bottom_foot_valid_neighbors[index] < self.num_tiles_wide and inchworm.bottom_foot_valid_neighbors[index + 1] < self.num_tiles_high):
                    pygame.draw.rect(screen, color, rect, 2)
            for index in range(0, len(inchworm.top_foot_valid_neighbors), 2):
                rect = self.getTileRect(inchworm.top_foot_valid_neighbors[index], inchworm.top_foot_valid_neighbors[index + 1])
                color = self.VALID_MOVE
                if inchworm.top_foot_valid_neighbors[index] > -1 and inchworm.top_foot_valid_neighbors[index + 1] > -1:
                    pygame.draw.rect(screen, color, rect, 2)


        for row in range(self.num_tiles_high):
            for col in range(self.num_tiles_wide):
                rect = self.getTileRect(col, row)
                color = self.OCCUPIED_TILE
                # rospy.loginfo(row * self.num_tiles_wide + col)
                if inchworm_occ[row * self.num_tiles_wide + col] == 1:
                    pygame.draw.rect(screen, color, rect, 2)

        # draw shingle depots
        pygame.draw.circle(screen, self.DEPOT_COLOR, (int(self.roof_margin/2), int(shingle_depots_pos[0] * self.tile_height_px + self.tile_height_px/2)), int(min(self.tile_height_px/2, self.roof_margin/2.5)))
        if len(shingle_depots_pos) > 1:
            pygame.draw.circle(screen, self.DEPOT_COLOR, (int(self.screen_width - self.roof_margin/2), int(shingle_depots_pos[1] * self.tile_height_px + self.tile_height_px/2)), int(min(self.tile_height_px/2, self.roof_margin/2.5)))

        # draw inchworms
        # rospy.loginfo(len(inchworms))
        for i, worm in enumerate(inchworms):
            # rospy.loginfo(f"drawing inchworm {i}")
            inchworm_id = worm.id
            bottom_foot_pos = worm.bottom_foot_pos
            bottom_foot_status = worm.bottom_foot_status
            top_foot_pos = worm.top_foot_pos
            top_foot_status = worm.top_foot_status
            self.draw_inchworm(screen, inchworm_id, bottom_foot_pos, bottom_foot_status, top_foot_pos, top_foot_status)


