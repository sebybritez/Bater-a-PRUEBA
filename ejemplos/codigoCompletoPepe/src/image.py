import cv2
import utils
import numpy as np
from enum import Enum
from vector import Vector
from decimal import Decimal
from fractions import Fraction

MIN_AREA = 100
MIN_COGNITIVE_AREA = 500

class ColourValues(Enum):
    """This class defines the RGB values for the colours that cognitive targets contain inside them (each possible ring colour)."""
    # In terms of RGB values
    BLACK = (0, 0, 0)
    RED = (207, 0, 0)
    BLUE =  (0, 0, 207)
    GREEN = (0, 207, 0)
    YELLOW = (207, 207, 0)

class ColourScores(Enum):
    """This class defines the scores for the colours that cognitive targets contain inside them (each possible ring colour)."""
    BLACK = -2
    RED = -1
    YELLOW = 0
    GREEN = 1
    BLUE = 2

class ImageProcessor:

    def __init__(self):
        self.last_token_position = Vector(10000, 10000)
        self.last_token_direction = Vector.ONE
        self.counter = 0
        self.debugging = False

    def debug_show(self, img, filename = "img"):
        """Displays the image for debugging purposes and saves it with the given filename."""
        cv2.imwrite(f'{filename}.png', img)
        cv2.imshow(filename, img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def clean_image(self, converted_img, thresh_value = 125):
        """It cleans the image in order to obtain de contour with children and without parent (hierarchy).
        Read this Opencv documentation: https://docs.opencv.org/4.x/d9/d8b/tutorial_py_contours_hierarchy.html"""
        if converted_img is None:
            return None, None
        aux_img = converted_img.copy()
        grey = cv2.cvtColor(converted_img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(grey, thresh_value, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours is None: return None, None
        if len(contours) == 1:
            """In case of only one contour, a color pixel count is done to discard already seen signs or false positives"""
            contour = contours[0]
            _, _, _, white_count, grey_count, _, _ = self.count_color_pixels(converted_img, contour)
            if grey_count >= white_count: return None, None
            return contour, thresh
        if hierarchy is None: return None, None
        contour = utils.get_target_contour(contours, hierarchy, False)
        if contour is None: return None, None
        return contour, thresh
    
    def is_yellow(self, b, g, r):
        """Checks if the given BGR values correspond to yellow color."""
        b = int(b)
        g = int(g)
        r = int(r)
        if b < 3 and r > 205 and r < 209 and g > 205 and g < 209: return True
        return False
    
    def is_red(self, b, g, r):
        """Checks if the given BGR values correspond to red color."""
        b = int(b)
        g = int(g)
        r = int(r)
        if g < 3 and b < 3 and r > 205 and r < 209: return True
        return False
    
    def is_blue(self, b, g, r):
        """Checks if the given BGR values correspond to blue color."""
        if r < 3 and g < 3 and b > 205 and b < 209: return True
        return False

    def is_green(self, b, g, r):
        """Checks if the given BGR values correspond to green color."""
        if r < 3 and b < 3 and g > 205 and g < 209: return True
        return False
    
    def is_grey(self, b, g, r):
        """Checks if the given BGR values correspond to grey color."""
        if b > 130 and g > 130 and r > 130 and b < 150 and g < 150 and r < 150: return True
        return False
    
    def is_white(self, b, g, r):
        """Checks if the given BGR values correspond to white color."""
        if b > 200 and g > 200 and r > 200: return True
        return False
    
    def is_black(self, b, g, r):
        """Checks if the given BGR values correspond to black color."""
        if b <= 50 and g <= 50 and r <= 50: return True
        return False
    
    def get_colour_score(self, name):
        """Returns the score associated with a given color name."""
        value = ColourScores[name].value
        return value
    
    def count_continuous_obs_pixels(self, img):
        """Counts the number of continuous pixels of the same color as the central pixel in the horizontal direction."""
        center_y ,center_x =  img.shape[0] // 2,  img.shape[1] // 2
        
        central_pixel = img[center_y, center_x]
        b_ref, g_ref, r_ref = central_pixel[:3]
        
        obstacle_px = 0

        if not self.is_exclude_color(b_ref, g_ref, r_ref):
            obstacle_px += 1

        #LEFT
        for x in range(center_x - 1, -1, -1):
            b, g, r = img[center_y, x][:3]
            
            if b == b_ref and g == g_ref and r == r_ref:
                if not self.is_exclude_color(b, g, r):
                    # print(f"LEFT: B {b}  G {g}  R {r}" )
                    obstacle_px += 1
            else:
                break
        #RIGHT
        for x in range(center_x + 1, img.shape[1]):
            b, g, r = img[center_y, x][:3]
            
            if b == b_ref and g == g_ref and r == r_ref:
                if not self.is_exclude_color(b, g, r):
                    # print(f"RIGHT: B {b}  G {g}  R {r}" )
                    obstacle_px += 1
            else:
                break
        return obstacle_px    

    def is_exclude_color(self, b, g, r):
        """Checks if the given BGR values correspond to excluded colors or specific conditions."""
        excluded_colors = {(207, 0, 0), (0, 207, 0), (0, 0, 207), (0, 207, 207), (221, 221, 221), (207, 207, 207), (213, 211, 210), (242, 240, 240), (128, 196, 196)} # Directions colors
        return (b, g, r) in excluded_colors or ((r < g) & (g < b) & (r < (g * 0.625)) & (r > ( g * 0.45)) & (g >= 0.9 * b)) or (b <= 3) & (g <= 3) & (r <= 3)
    
    def clean_walls(self, img):
        """It cleans the walls of the image, leaving only the crucial colors for the victim detection."""
        
        if img is None: return None
        if img.size == 0: return None

        b = img[...,0]
        g = img[...,1]
        r = img[...,2]

        is_wall = (r < g) & (g < b) & (r < (g * 0.625)) & (r > ( g * 0.45)) & (g >= 0.9 * b)
        channels = img.shape[2] if img.ndim == 3 else 1
        fill_black = (0, 0, 0, 255) if channels == 4 else (0, 0, 0)

        aux_img = img.copy()
        aux_img[is_wall] = fill_black
        return aux_img
    
    def thresh_with_given_colour(self, img, color):
        """It thresholds the image, setting as white the pixels that match the given color and as black the rest."""
        if img is None: return None
        if img.size == 0: return None

        b = img[...,0]
        g = img[...,1]
        r = img[...,2]

        values = color.value
        given_r, given_g, given_b = values[0], values[1], values[2]

        is_color = (b == given_b) & (g == given_g) & (r == given_r)

        channels = img.shape[2] if img.ndim == 3 else 1
        fill_white = (255, 255, 255, 255) if channels == 4 else (255, 255, 255)
        fill_black = (0, 0, 0, 255) if channels == 4 else (0, 0, 0)

        aux_img = np.full_like(img, fill_black, dtype=np.uint8)
        aux_img[is_color] = fill_white

        grey = cv2.cvtColor(aux_img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(grey, 125, 255, cv2.THRESH_BINARY)
        return thresh

    def threshold_image(self, img):
        """An own thresh that is adptative to the simulation's noise, setting as white crucial color pixels
        and as black the not crucial ones for the victims detection. NumPy operartors are used due to its efficiency 
        (faster than Python loops)."""
        if img is None: return None
        if img.size == 0: return None

        b = img[...,0]
        g = img[...,1]
        r = img[...,2]

        is_wall = (r < g) & (g < b) & (r < (g * 0.625)) & (r > ( g * 0.45)) & (g >= 0.9 * b)
        is_black = (b <= 50) & (g <= 50) & (r <= 50)
        is_not_crucial = is_wall | is_black # Is crucial: yellow, red, white

        channels = img.shape[2] if img.ndim == 3 else 1
        fill_white = (255, 255, 255, 255) if channels == 4 else (255, 255, 255)
        fill_black = (0, 0, 0, 255) if channels == 4 else (0, 0, 0)

        aux_img = np.full_like(img, fill_white, dtype=np.uint8)
        aux_img[is_not_crucial] = fill_black

        grey = cv2.cvtColor(aux_img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(grey, 125, 255, cv2.THRESH_BINARY)
        # self.debug_show(thresh, "thresh")
        return thresh

    def threshold_cognitive_image(self, img):
        """An own thresh that is adptative to the simulation's noise, setting as white crucial color pixels
        and as black the not crucial ones for cognitive targets detection. NumPy operartors are used due to its 
        efficiency (faster than Python loops)."""
        if img is None: return None
        if img.size == 0: return None

        b = img[...,0]
        g = img[...,1]
        r = img[...,2]

        is_wall = (r < g) & (g < b) & (r < (g * 0.625)) & (r > ( g * 0.45)) & (g >= 0.9 * b)
        is_white = (b > 190) & (g > 190) & (r > 190)
        is_sky = (r < g) & (g < b) & (g < b * 0.76) & (g > b * 0.60)
        is_not_crucial = is_wall | is_white | is_sky # | is_black # Is crucial: yellow, red, white ADDED: blue, green

        is_red = (r > 205) & (r < 209) & (g < 3) & (b < 3)
        is_yellow = (r > 205) & (r < 209) & (g > 205) & (g < 209) & (b < 3)
        is_blue = (b > 205) & (b < 209) & (g < 3) & (r < 3)
        is_green = (g > 205) & (g < 209) & (r < 3) & (b < 3)
        is_black = (b <= 50) & (g <= 50) & (r <= 50)
        is_crucial = is_red | is_yellow | is_blue | is_green | is_black

        channels = img.shape[2] if img.ndim == 3 else 1
        fill_white = (255, 255, 255, 255) if channels == 4 else (255, 255, 255)
        fill_black = (0, 0, 0, 255) if channels == 4 else (0, 0, 0)

        # aux_img = np.full_like(img, fill_white, dtype=np.uint8)
        # aux_img[is_not_crucial] = fill_black

        aux_img = np.full_like(img, fill_black, dtype=np.uint8)
        aux_img[is_crucial] = fill_white
        aux_img[is_wall] = fill_black

        grey = cv2.cvtColor(aux_img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(grey, 125, 255, cv2.THRESH_BINARY)
        # self.debug_show(thresh, "thresh")
        return thresh
    
    def get_contour_centre(self, img, analyzing_after_placing = False, is_zoomed_img = False, is_cognitive = False):
        """It returns the centre of the contour, if it is not found, it returns None."""
        if not is_cognitive:
            contour, _ = self.clean_image_with_own_thresh(img, is_zoomed_img = is_zoomed_img, debug_show = False)
            # print(f"Is contour None? {contour is None}")
        else:
            # print(f"Is cognitive, using different cleaning image!")
            contour, _ = self.clean_cognitive_image_with_own_thresh(img)
        if contour is None and analyzing_after_placing:
            if not is_cognitive:
                thresh = self.threshold_image(img)
            else:
                thresh = self.threshold_cognitive_image(img)
            contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contour = self.select_biggest_contour(contours)
            if contour is None: return None
        
        contour_centre = self.obtain_contours_centre(contour)
        # print(f"Contour centre: {contour_centre}")
        return contour_centre
        
    def clean_image_with_own_thresh(self, converted_img, analyzing_after_placing = False, is_zoomed_img = False, debug_show = False):
        """Same logic that clean_image but using our own thresh, this step is used when analyzing the complete image and
        when using the is_image, to detect if the image is a sign or not."""
        aux_img = converted_img.copy()
        if converted_img is None:
            return None, None
        thresh = self.threshold_image(converted_img)
        if debug_show:
            self.debug_show(thresh, "thresh")
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours is None: return None, None

        # print(f"Contours {len(contours)}")
        """If theres no hierarchy to analyze (only one contour), we skip the hierarchy-based contour selection"""
        if len(contours) == 1:
            # print("ACAACA")
            contour = contours[0]
            contour = self.discard_contour(converted_img, contour, is_zoomed_img = is_zoomed_img)
            if contour is None: return None, None
            if debug_show:
                aux_img = cv2.drawContours(aux_img, [contour], -1, (0, 255, 0), 1)
                self.debug_show(aux_img, "contour")
            return contour, thresh
        
        if hierarchy is None: return None, None
        contour = utils.get_target_contour(contours, hierarchy, False)
        if contour is None: 
            biggest_contour = self.select_biggest_contour(contours)
            if debug_show:
                cv2.drawContours(aux_img, [biggest_contour], -1, (0, 255, 0), 1)
                self.debug_show(aux_img, "biggest_contour")
            contour = self.discard_contour(converted_img, biggest_contour, is_zoomed_img = is_zoomed_img)
            if contour is None and not analyzing_after_placing:
                return None, None
        return contour, thresh
        
        
    def clean_cognitive_image_with_own_thresh(self, converted_img, analyzing_after_placing = False):
        """Same logic that clean_image but using our own thresh, this step is used when analyzing the complete image and
        when using the is_image, to detect if the image is a sign or not."""
        aux_img = converted_img.copy()
        if converted_img is None:
            return None, None
        thresh = self.threshold_cognitive_image(converted_img)
        # self.debug_show(thresh, "thresh")

        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours is None: return None, None

        # print(f"Contours {len(contours)}")
        """If theres no hierarchy to analyze (only one contour), we skip the hierarchy-based contour selection"""
        if len(contours) == 1:
            # print(f"Theres only one contour, hierarchy: {hierarchy}")
            # print("ACAACA")
            contour = contours[0]
            contour = self.discard_contour(converted_img, contour)
            if contour is None: return None, None
            # aux_img = cv2.drawContours(aux_img, [contour], -1, (0, 255, 0), 1)
            # self.debug_show(aux_img, "contour")
            return contour, thresh
        
        if hierarchy is None: return None, None
        contour = utils.get_target_contour(contours, hierarchy, False)
        if contour is None:
            biggest_contour = self.select_biggest_contour(contours)
            cv2.drawContours(aux_img, [biggest_contour], -1, (0, 255, 0), 1)
            # self.debug_show(aux_img, "biggest_contour")
            contour = self.discard_contour(converted_img, biggest_contour)
            if contour is None and not analyzing_after_placing:
                return None, None
        return contour, thresh
    
    def clean_contour(self, contour, img):
        """Cleans the contour, obtaining the four corners of the contour and returning a cleaned image with only the area of the contour."""
        if contour is None: return None
        if img is None: return None

        aux_img = img.copy()
        pts = contour.reshape(-1, 2) # Reshape the contour to a 2D array of points

        top_left_corner = pts[np.argmin(pts.sum(axis=1))] # The top-left corner will have the smallest sum of x and y coordinates
        top_right_corner = pts[np.argmin(np.diff(pts, axis=1))] # The top-right corner will have the smallest difference between x and y coordinates
        bottom_right_corner = pts[np.argmax(pts.sum(axis=1))] # The bottom-right corner will have the largest sum of x and y coordinates
        bottom_left_corner = pts[np.argmax(np.diff(pts, axis=1))] # The bottom-left corner will have the largest difference between x and y coordinates
        # print(f"Top left corner: {top_left_corner}, Top right corner: {top_right_corner}, Bottom left corner: {bottom_left_corner}, Bottom right corner: {bottom_right_corner}")

        aux_img = cv2.circle(aux_img, top_left_corner, 1, (255, 0, 0), -1)
        aux_img = cv2.circle(aux_img, top_right_corner, 1, (0, 255, 0), -1)
        aux_img = cv2.circle(aux_img, bottom_left_corner, 1, (0, 0, 255), -1)
        aux_img = cv2.circle(aux_img, bottom_right_corner, 1, (255, 0, 255), -1)
        # self.debug_show(aux_img, "corners")

        if top_left_corner[1] > top_right_corner[1]:
            top_right_corner = (top_right_corner[0], top_left_corner[1])
        else:
            top_left_corner = (top_left_corner[0], top_right_corner[1])

        if bottom_left_corner[1] < bottom_right_corner[1]:
            bottom_right_corner = (bottom_right_corner[0], bottom_left_corner[1])
        else:
            bottom_left_corner = (bottom_left_corner[0], bottom_right_corner[1])

        if top_left_corner[0] > bottom_left_corner[0]:
            bottom_left_corner = (top_left_corner[0], bottom_left_corner[1])
        else:
            top_left_corner = (bottom_left_corner[0], top_left_corner[1])

        if top_right_corner[0] < bottom_right_corner[0]:
            bottom_right_corner = (top_right_corner[0], bottom_right_corner[1])
        else:
            top_right_corner = (bottom_right_corner[0], top_right_corner[1])

        cleaned_img = img[top_left_corner[1]:bottom_left_corner[1], top_left_corner[0]:top_right_corner[0]]
        cleaned_contour = np.array([top_left_corner, top_right_corner, bottom_right_corner, bottom_left_corner])
        return cleaned_img, cleaned_contour
    
    def discard_contour(self, converted_img, contour, is_zoomed_img = False):
        """Discards contours thtat are from signs already detected based on the color pixel counts and relations between them."""
        yellow_count, red_count, white_count, black_count, grey_count, blue_count, green_count = self.count_color_pixels(converted_img, contour)
        if (grey_count >= white_count) and (yellow_count == 0 and red_count == 0 and blue_count == 0 and green_count == 0): # Victim already seen
            return None
        if (yellow_count >= 1 or red_count >= 1 or blue_count >= 1 or green_count >= 1) and (white_count != 0): # A cognitive with white for us is a cognitive already seen
            return None
        if (white_count != 0) and (black_count == 0) and not is_zoomed_img:
            return None
        return contour
    
    def clean_letter(self, img, thresh):
        """It looks for the contour with parent and no children."""
        aux_img = img.copy()
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if hierarchy is None: return None
        contour = utils.get_target_contour(contours, hierarchy, True)
        if contour is None: return None
        return contour
    
    def select_biggest_contour(self, contours):
        """Selects the biggest contour from a list of contours."""
        if contours is None or len(contours) == 0: return None
        biggest_contour = max(contours, key=cv2.contourArea)
        return biggest_contour
    
    def count_black_white_pixels(self, img):
        """Returns the amount of black and white pixels in the image."""
        black_pixels = np.count_nonzero(img == 0)
        white_pixels = np.count_nonzero(img == 255)
        return black_pixels, white_pixels

    def squares_pixel_count(self, upper_square, middle_square, lower_square, left_square, right_square, upper_left_square, upper_right_square, lower_left_square, lower_right_square):
        """Splits the letter contour in 9 equal squares that are used to analize the letter."""
        upper_pixels = self.get_pixels_percentages(upper_square)
        middle_pixels = self.get_pixels_percentages(middle_square)
        lower_pixels = self.get_pixels_percentages(lower_square)
        left_pixels = self.get_pixels_percentages(left_square)
        right_pixels = self.get_pixels_percentages(right_square)
        upper_left_pixels = self.get_pixels_percentages(upper_left_square)
        upper_right_pixels = self.get_pixels_percentages(upper_right_square)
        lower_left_pixels = self.get_pixels_percentages(lower_left_square)
        lower_right_pixels = self.get_pixels_percentages(lower_right_square)

        return upper_pixels, middle_pixels, lower_pixels, left_pixels, right_pixels, upper_left_pixels, upper_right_pixels, lower_left_pixels, lower_right_pixels

    def rotate_image(self, img, angle):
        """Rotates the image to a given angle."""
        height, width = img.shape[0], img.shape[1]
        M = cv2.getRotationMatrix2D((width / 2, height / 2), angle, 1)
        rotated_image = cv2.warpAffine(img, M, (width, height))
        contour, _ = self.clean_image_with_own_thresh(rotated_image)
        if contour is None: return None
        return rotated_image
    
    def straighten_image(self, img, contour):
        """Straightens the image based on the contour's convex hull."""
        if contour is None: return None
        hull = cv2.convexHull(contour)
        # Enderezar la imagen usando los puntos del casco convexos
        src_pts = np.float32(self.get_contour_corners(hull))

        epsilon = 0.01 * cv2.arcLength(hull, closed = True)
        approx = cv2.approxPolyDP(hull, epsilon, closed = True)
        if approx is not None and len(approx) == 4:
            for factor in [0.03, 0.05, 0.07, 0.1]:
                epsilon = factor * cv2.arcLength(hull, closed=True)
                approx = cv2.approxPolyDP(hull, epsilon, closed=True)
                if len(approx) == 4:
                    break

        x, y, w, h = cv2.boundingRect(contour)
        dst_pts = np.float32([
            [x, y ], # Top-left corner
            [x + w - 1, y], # Top-right corner
            [x + w - 1, y + h - 1], # Bottom-right corner
            [x, y + h - 1], # Bottom-left corner
        ])

        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))
        return warped

    def return_victim_letter(self, img, contour):
        """Victims detection logic"""
        if contour is None: return None
        if img is None: return None
        aux_img = img.copy()

        # 1) Rotate the image if the contour's angle is not a multiple of 90 degrees, otherwise, we keep the original image        
        angle = cv2.minAreaRect(contour)[2]
        if not utils.near_multiple(angle, 90, 2):
            rotated_image = self.rotate_image(img, angle)
            if rotated_image is None: return None
        else: rotated_image = img.copy()

        # 2) Clean the walls of the image and then obtain the new letter contour and the new thresholded image.
        image_with_no_wall = self.clean_walls(rotated_image)
        raw_letter_contour, thresh = self.clean_image(image_with_no_wall, 100)

        # 3) Straighten the image if necessary (contour is considerably deformed). If straightened, clean again the image.
        if self.needs_contour_be_straightened(raw_letter_contour):
            image_with_no_wall = self.straighten_image(image_with_no_wall, raw_letter_contour)
            raw_letter_contour, thresh = self.clean_image(image_with_no_wall, 100)

        # 4) Clean the contour to obtain the four corners of the letter and crop the image to only have the letter.
        if thresh is None: return None
        _, letter_contour = self.clean_contour(raw_letter_contour, image_with_no_wall)
        if letter_contour is None: return None

        # 5) Split the letter contour in 9 (as a tic-tac-toe) squares and count the black and white pixels in each square.
        squares = self.get_letter_regions(thresh, letter_contour)
        if squares is None: return None
        upper_square, middle_square, lower_square, left_square, right_square, upper_left_square, upper_right_square, lower_left_square, lower_right_square = squares
        upper_pixels, middle_pixels, lower_pixels, left_pixels, right_pixels, upper_left_pixels, upper_right_pixels, lower_left_pixels, lower_right_pixels  = self.squares_pixel_count(upper_square, middle_square, lower_square, left_square, right_square, upper_left_square, upper_right_square, lower_left_square, lower_right_square)
 
        # 6) Based on the pixel counts, we determine the letter (U, H, S) that is represented by the contour.
        letter = self.get_letter(upper_pixels, middle_pixels, lower_pixels, left_pixels, right_pixels, upper_left_pixels, upper_right_pixels, lower_left_pixels, lower_right_pixels)
        if letter is None: return None
        return letter

    def get_letter(self, upper_pixels, middle_pixels, lower_pixels, left_pixels, right_pixels, upper_left_pixels, upper_right_pixels, lower_left_pixels, lower_right_pixels):
        """Determines the letter (U, H, S) basing on the amount of white pixels in the middle square and the relation between black and white
        pixels of each top/bottom corner square."""
        middle_white_pixels = middle_pixels["White"] if middle_pixels is not None and "White" in middle_pixels else 0
        upper_left_black_pixels = upper_left_pixels["Black"] if upper_left_pixels is not None and "Black" in upper_left_pixels else 0
        upper_right_black_pixels = upper_right_pixels["Black"] if upper_right_pixels is not None and "Black" in upper_right_pixels else 0
        lower_left_black_pixels = lower_left_pixels["Black"] if lower_left_pixels is not None and "Black" in lower_left_pixels else 0
        lower_right_black_pixels = lower_right_pixels["Black"] if lower_right_pixels is not None and "Black" in lower_right_pixels else 0
        
        if middle_white_pixels > 90: return 'U'
        black_pixels = sorted([upper_left_black_pixels, upper_right_black_pixels, lower_left_black_pixels, lower_right_black_pixels], reverse = True)
        biggest_black_average = (black_pixels[0] + black_pixels[1]) / 2
        smallest_black_average = (black_pixels[2] + black_pixels[3]) / 2
        diff = abs(biggest_black_average - smallest_black_average)
        if diff <= 15: return 'H'
        return 'S'

    def get_pixels_percentages(self, region):
        """Returns the white and black percentages from a dict."""
        pixels = {}
        black_pixels, white_pixels = self.count_black_white_pixels(region)
        black_pix_per, white_pix_per = utils.get_percentage(black_pixels, white_pixels)
        if black_pix_per is None: return None
        if white_pix_per is None: return None
        if black_pix_per == 0 and white_pix_per == 0: return None
        pixels["Black"] = black_pix_per
        pixels["White"] = white_pix_per
        if pixels is None: return None
        return pixels

    def get_letter_regions(self, thresh, letter_contour):
        """Slices the letter contour in 5 regions (upper, middle, lower, left and right) to analize the letter."""
        if thresh is None: return None
        copy = thresh.copy()
        x, y, width, height = cv2.boundingRect(letter_contour)
 
        rect = thresh[y: y + height, x: x + width]
        
        if abs(rect.shape[0] - rect.shape[1]) > 10: return None
        if rect.shape[0] * rect.shape[1] == 0: return None

        black_pixels = np.count_nonzero(rect == 0)
        if black_pixels == 0: return None
 
        upper_square = thresh[y: y + int(height / 3), x + int(width / 3): x + int(2 * (width / 3))]
        middle_square = thresh[y + int(height / 3): y + int(2 * (height / 3)),x + int(width / 3): x + int(2 * (width / 3))]
        lower_square = thresh[y + int(2 * (height / 3)): y + height, x + int(width / 3): x + int(2 * (width / 3))]
        left_square = thresh[y + int(height / 3): y + int(2 * (height / 3)), x:x + int(width / 3)]
        right_square = thresh[y + int(height / 3): y + int(2 * (height / 3)), x + int(2 * (width / 3)): x + width]
        upper_left_square = thresh[y: y + int(height / 3), x: x + int(width / 3)]
        upper_right_square = thresh[y: y + int(height / 3), x + int(2 * (width / 3)): x + width]
        lower_left_square = thresh[y + int(2 * (height / 3)): y + height, x: x + int(width / 3)]
        lower_right_square = thresh[y + int(2 * (height / 3)): y + height, x + int(2 * (width / 3)): x + width]

        return upper_square, middle_square, lower_square, left_square, right_square, upper_left_square, upper_right_square, lower_left_square, lower_right_square    

    def get_min_circle_contour(self, img, r, g, b):
        """Obstains the first concentric circle contour of the cognitive target (central ring)."""
        if (r, g, b) not in ColourValues._value2member_map_:
            print(f"Color values: R={r}, G={g}, B={b} not found in ColourValues enum")
            return None
        
        min_circle_tresh = self.thresh_with_given_colour(img, ColourValues((r, g, b)))
        if min_circle_tresh is None: return None

        min_circle_contours, hierarchy = cv2.findContours(min_circle_tresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if min_circle_contours is None or len(min_circle_contours) == 0: return None
        if len(min_circle_contours) == 1:
            min_circle_contour = min_circle_contours[0]
        else:
            min_circle_contour = utils.get_target_contour(min_circle_contours, hierarchy, is_letter_contour=True)
        return min_circle_contour
    

    def get_analyzing_side_data(self, contour, cx):
        """Determines which side of the contour is bigger and better for analyzing. Returns a boolean for 
        each side and the radius of the contour."""
        count_right_side = False
        count_left_side = False

        rightmost = tuple(contour[contour[:,:,0].argmax()][0])
        leftmost = tuple(contour[contour[:,:,0].argmin()][0])

        if self.contour_on_left_border(contour):
            count_right_side = True
        elif self.contour_on_right_border(contour):
            count_left_side = True
        else:
            right_diff = abs(rightmost[0] - cx)
            left_diff = abs(leftmost[0] - cx)
            if right_diff > left_diff:
                count_right_side = True
            else:
                count_left_side = True

        if count_left_side:
            radius = cx - leftmost[0]
        else:
            radius = rightmost[0] - cx

        return count_left_side, count_right_side, radius
    
    def get_key_points_data(self, img, radius, cx, cy, count_left_side, count_right_side):
        """Obtains the five key points of the cognitive target to analyze its color values."""
        step = radius / 5
        points = []
        for i in range(5):
            point_data = {}
            if count_left_side:
                x = int((cx - radius) + (i + 0.5) * step)
            elif count_right_side:
                x = int((cx + radius) - (i + 0.5) * step)
            point = (x, int(cy))
            pixel = img[point[1], point[0]]
            b, g, r = pixel[:3]
            point_data[point] = (r, g, b)
            points.append(point_data)
        return points
    
    def get_sum_values(self, points):
        """Calculates the sum of colour scores for the given points."""
        sum_value = 0
        for point_data in points:
            for value in point_data.values():
                r, g, b = value
                if (r, g, b) not in ColourValues._value2member_map_:
                    continue
                colour_name_in_enum = ColourValues((r, g, b)).name
                value = self.get_colour_score(colour_name_in_enum)
                sum_value += value 
        return sum_value

    def get_cognitive_letter(self, count):
        """Determines the cognitive letter based on the sum of colour scores."""
        if count == 0:
            return 'F'
        elif count == 1:
            return 'P'
        elif count == 2:
            return 'C'
        elif count == 3:
            return 'O'
        else:
            return 'Z' # NOTE (Martu): Fake signs representation
        
    def return_cognitive_letter(self, img):
        """Cognitive target detection logic"""
        if img is None: return None
        aux_img = img.copy()

        # 1) Clean the image to obtain the contour of the cognitive target.
        contour, thresh = self.clean_cognitive_image_with_own_thresh(img)
        if contour is None: return None

        # 2) Verify if the sign is simetrical enough to be analyzed.
        rect = cv2.minAreaRect(contour)
        if rect is None: return None
        (x, y), (w, h), angle = rect
        print(f"Diff between width and height: {abs(w - h)}")
        if abs(w - h) > 8: return None

        # 3) Obtain the minimum enclosing circle of the contour to determine its center and radius.
        circ = cv2.minEnclosingCircle(contour)
        if circ is None: return None
        center, radius = circ

        # 4) Get the color values of the center pixel of the cognitive target.
        center_b = img[int(center[1]), int(center[0]), 0]
        center_g = img[int(center[1]), int(center[0]), 1]
        center_r = img[int(center[1]), int(center[0]), 2]

        # 5) Obtain the contour of the minimum enclosing circle of the cognitive target based on the center color values.
        min_circle_contour = self.get_min_circle_contour(img, center_r, center_g, center_b)
        if min_circle_contour is None: return None

        # 6) Get the bounding rectangle of the minimum enclosing circle contour to determine its exact center.
        rect = cv2.boundingRect(min_circle_contour)
        if rect is None: return None
        x, y, w, h = rect # Corners, width and height of the bounding rectangle

        cx, cy = x + w / 2, y + h / 2

        # 7) Determine which side of the contour is bigger and better for analyzing.
        count_left_side, count_right_side, radius = self.get_analyzing_side_data(contour, cx)

        # 8) Obtain the five key points of the cognitive target to analyze its color values.
        points = self.get_key_points_data(img, radius, cx, cy, count_left_side, count_right_side)

        # 9) Calculate the sum of color scores for the given points.
        sum_value = self.get_sum_values(points)

        cognitive_letter = self.get_cognitive_letter(sum_value)
        return cognitive_letter


    def count_color_pixels(self, img, contour):
        """Counts the color pixels inside a contour, returns the count of yellow, red, white, black and grey pixels."""
        if contour is None: return None
        if img is None: return None

        area = cv2.contourArea(contour)
        # print(f"Contour area: {area}")
        if area < MIN_AREA or len(contour) < 3:
            return 0, 0, 0, 0, 0, 0, 0
        yellow_count, red_count, black_count, white_count, grey_count, blue_count, green_count = 0, 0, 0, 0, 0, 0, 0
        for x in range(img.shape[0]):
            for y in range(img.shape[1]):
                if cv2.pointPolygonTest(contour, (x, y), False) > 0:
                    pixel = img[y, x]
                    b, g, r = pixel[:3]
                    if self.is_white(b, g, r):
                        white_count += 1
                    elif self.is_black(b, g, r):
                        black_count += 1
                    elif self.is_grey(b, g, r):
                        grey_count += 1
                    elif self.is_red(b, g, r):
                        red_count += 1
                        # print(f"Pixel at ({x}, {y}): B={b}, G={g}, R={r}")
                    elif self.is_yellow(b, g, r):
                        yellow_count += 1
                        # print(f"Pixel at ({x}, {y}): B={b}, G={g}, R={r}")
                    elif self.is_blue(b, g, r):
                        blue_count += 1
                    elif self.is_green(b, g, r):
                        green_count += 1
        return yellow_count, red_count, white_count, black_count, grey_count, blue_count, green_count

    def contour_on_lateral_border(self, cnt):
        """Checks if a contour is touching any lateral border."""
        leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])
        rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])
        if leftmost[0] == 0 or leftmost[0] == 39: return True
        if rightmost[0] == 0 or rightmost[0] == 39: return True
        return False

    def contour_on_left_border(self, cnt):
        """Checks if a contour is touching the left border."""
        leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])
        if leftmost[0] == 0 or leftmost[0] == 39: return True
        return False
    
    def contour_on_right_border(self, cnt):
        """Checks if a contour is touching the right border."""
        rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])
        if rightmost[0] == 0 or rightmost[0] == 39: return True
        return False   
    
    def count_image_color_pixels(self, img):
        """Counts the color pixels in the image, returns the count of yellow, red, white, black, grey and wall pixels."""
        if img is None or img.size == 0:
            return None

        b = img[..., 0]
        g = img[..., 1]
        r = img[..., 2]

        is_wall = (((b > 125) & (b < 165) & (g > 115) & (g < 150) & (r > 55) & (r < 75)) | ((b > 25) & (b < 65) & (g > 25) & (g < 65) & (r > 15) & (r < 45))) # | ((b > 27) & (b < 37) & (g > 25) & (g < 35) & (r > 13) & (r < 23)))
        is_white = (b > 200) & (g > 200) & (r > 200)
        is_black = (b <= 25) & (g <= 25) & (r <= 25)
        is_grey = (b > 130) & (g > 130) & (r > 130) & (b < 150) & (g < 150) & (r < 150) # | (b - r < 5) & (g - r < 5) & (b - g < 5)
        # NOTE (Martu): For no detecting obstacles and/or redetecting!
        is_red = (b < 5) & (g < 5) & (r > 205) & (r < 209)
        is_yellow = (b < 5) & (g > 205) & (r > 205) & (g < 209) & (r < 209)
        is_green = (b < 5) & (g > 205) & (r < 5) & (g < 209)
        is_blue = (b > 205) & (g < 5) & (r < 5) & (b < 209)

        yellow_count = np.count_nonzero(is_yellow)
        red_count = np.count_nonzero(is_red)
        white_count = np.count_nonzero(is_white)
        black_count = np.count_nonzero(is_black)
        grey_count = np.count_nonzero(is_grey)
        wall_count = np.count_nonzero(is_wall)
        blue_count = np.count_nonzero(is_blue)
        green_count = np.count_nonzero(is_green)

        return yellow_count, red_count, white_count, black_count, grey_count, wall_count, blue_count, green_count
    
    def is_image_zoomed_in(self, img):
        """Evaluates if the image is or not a zoomed in sign."""
        yellow_count, red_count, white_count, black_count, _, wall_count, blue_count, green_count = self.count_image_color_pixels(img)
        if yellow_count == 0 and red_count == 0 and black_count == 0 and white_count == 0 == 0 and blue_count == 0 and green_count == 0: return False
        # print(f"yellow: {yellow_count}, red: {red_count}, white: {white_count}, black: {black_count}, blue: {blue_count}, green: {green_count}, wall: {wall_count}")
        pixels = yellow_count + red_count + black_count + white_count + blue_count + green_count
        color_pixels = sorted([yellow_count, red_count, black_count, white_count, blue_count, green_count], reverse = True)
        wall_percentage = utils.get_percentage(pixels, wall_count)[1]
        # print(f"wall percentage: {wall_percentage}")
        if wall_percentage is None: return False
        if wall_percentage > 80: return False
        if color_pixels[0] < 500: return False

        # NOTE (Martu): For no redetecting!
        if white_count > 1 and black_count == 0: return False
        if white_count >= 200 and black_count >= white_count * 0.4: return False
        # if grey_count > 1: return False
        # if white_count > 1 and black_count == 0: return False
        # if white_count > 1 and (red_count != 0 or yellow_count != 0 or blue_count != 0 or green_count != 0): return False
        if black_count > 1 and white_count == 0 and red_count == 0 and yellow_count == 0: return False # Si no hay blanco pero tampoco hay ni rojo ni amarillo (o sea, no es F/O)
        if red_count > 1 and white_count == 0 and yellow_count == 0: return False # Si no hay blanco pero sí rojo, y no hay amarillo (o sea, no es O ni F)
        if yellow_count > 1 and red_count == 0: return False # Si hay amarillo, pero no hay rojo, al no ser una O; no puede ser nada
        if red_count > 1 and black_count > 1 and yellow_count == 0: return False # Si hay rojo pero no amarillo, o sea que es F y no O, no puede tener negro
        if yellow_count > 1 and red_count > 1 and white_count > 1: return False # Si es O, no puede tener blanco
        if black_count > 1 and white_count > 1 and (red_count > 1 or yellow_count > 1): return False # Si es P/C/S/H/U, no puede tener ni rojo ni amarillo
        return True
    
    def is_zoomed_image_a_victim(self, img):
        """Evaluates if the zoomed image is a victim or not."""
        if img is None: return False
        yellow_count, red_count, white_count, black_count, grey_count, wall_count, blue_count, green_count = self.count_image_color_pixels(img)
        if yellow_count > 1 or red_count > 1 or blue_count > 1 or green_count > 1: return False
        return True
    
    def contour_on_top_or_bottom_border(self, cnt):
        """Checks if a contour is touching the top or bottom border."""
        topmost = tuple(cnt[cnt[:,:,1].argmin()][0])
        bottommost = tuple(cnt[cnt[:,:,1].argmax()][0])
        if topmost[1] == 0 or topmost[1] == 39: return True
        if bottommost[1] == 0 or bottommost[1] == 39: return True
        return False
    
    def needs_contour_be_straightened(self, contour):
        """Evaluates if the contour is deformed and needs to be straightened."""
        if contour is None: return False
        contour_area = cv2.contourArea(contour)
        _, _, w, h = cv2.boundingRect(contour)
        bounding_rect_area = w * h
        if bounding_rect_area == 0: return False
        area_ratio = contour_area / bounding_rect_area
        print(f"Contour area: {contour_area}, Bounding rect area: {bounding_rect_area}, Area ratio: {area_ratio}")
        if area_ratio < 0.9: # If the contour area is significantly smaller than the bounding rectangle area, it might be transformed
            return True
        return False
    
    def obtain_contours_centre(self, cnt):
        """Returns the centre of a given contour."""
        # print(f" Contour {cnt}")
        if cnt is None or len(cnt) == 0: return None
        contour_centre = cv2.minAreaRect(cnt)[0][0]
        return contour_centre
    
    def get_contour_corners(self, contour):
        """Obtains the four corners of a contour."""
        if contour is None or len(contour) == 0: return None
        pts = contour.reshape(-1, 2) # Reshape the contour to a 2D array of points

        top_left_corner = pts[np.argmin(pts.sum(axis = 1))] # The top-left corner will have the smallest sum of x and y coordinates
        top_right_corner = pts[np.argmin(np.diff(pts, axis = 1))] # The top-right corner will have the smallest difference between x and y coordinates
        bottom_right_corner = pts[np.argmax(pts.sum(axis = 1))] # The bottom-right corner will have the largest sum of x and y coordinates
        bottom_left_corner = pts[np.argmax(np.diff(pts, axis = 1))] # The bottom-left corner will have the largest difference between x and y coordinates
        return top_left_corner, top_right_corner, bottom_right_corner, bottom_left_corner
    
    def is_analizable(self, contour, tol = 8):
        """Compares the hieght and width of a contour to determine if it is analizable or not."""
        _, _, w, h = cv2.boundingRect(contour)
        # print(f"Contour width: {w}, height: {h}, difference: {abs(w - h)}")
        if abs(w - h) > tol: return False
        return True

    def is_victim(self, img,  analyzing_after_placing = False):
        """Evaluates if the image is an unseen sign or not."""
        if img is None: return False
        if img.size == 0: return False
        contour, _ = self.clean_image_with_own_thresh(img,  analyzing_after_placing)
        if contour is None: return False
        yellow_count, red_count, white_count, black_count, grey_count, blue_count, green_count = self.count_color_pixels(img, contour)
        if yellow_count > 1 or red_count > 1 or blue_count > 1 or green_count > 1: return False

        # NOTE (Martu): For no obstacles detecting and/or redetecting!
        if white_count == 0 or black_count == 0: return False

        if len(contour) < 3: return False
        contour_on_top_or_bottom_border = self.contour_on_top_or_bottom_border(contour)
        if contour_on_top_or_bottom_border: 
            is_analizable = self.is_analizable(contour, tol = 8)
            if not is_analizable: return False
        contour_on_lateral_border = self.contour_on_lateral_border(contour)
        if contour_on_lateral_border: 
            if not self.is_analizable(contour, tol = 8): return False
        contour_area = cv2.contourArea(contour)
        if contour_area < MIN_AREA: return False
        is_analizable = self.is_analizable(contour, tol = 8)
        if not is_analizable: return False
        return True
    
    def is_cognitive(self, img,  analyzing_after_placing = False):
        """Evaluates if the image is an unseen sign or not."""
        if img is None: return False
        if img.size == 0: return False
        contour, _ = self.clean_cognitive_image_with_own_thresh(img,  analyzing_after_placing)
        if contour is None: 
            return False
        yellow_count, red_count, white_count, black_count, _, blue_count, green_count = self.count_color_pixels(img, contour)
        # print(f"Yellow count: {yellow_count}, Red count: {red_count}, Black count: {black_count}, Blue count: {blue_count}, Green count: {green_count}")
        if yellow_count == 0 and red_count == 0 and black_count == 0 and blue_count == 0 and green_count == 0: return False
        # print(f"White count: {white_count}")
        if white_count != 0: 
            # print(f"White pixels found in cognitive image, count: {white_count}")
            return False
        if len(contour) < 3: 
            return False
        contour_on_top_or_bottom_border = self.contour_on_top_or_bottom_border(contour)
        if contour_on_top_or_bottom_border: 
            is_analizable = self.is_analizable(contour, tol = 8)
            if not is_analizable: 
                return False
        contour_on_lateral_border = self.contour_on_lateral_border(contour)
        if contour_on_lateral_border: 
            if not self.is_analizable(contour, tol = 8): 
                return False
        contour_area = cv2.contourArea(contour)
        if contour_area < MIN_COGNITIVE_AREA: 
            return False
        is_analizable = self.is_analizable(contour, tol = 8)
        if not is_analizable: 
            return False
        # print(f"Cognitive contour area: {contour_area}") 
        return True
    
    def analyze_zoomed_image(self, img, last_position, last_direction):
        """Zoomed images analysis logic."""
        aux_img = img.copy()
        if last_position.distance_to(self.last_token_position) < 0.015 and self.last_token_direction.dot(last_direction) > 0.9:
            return None
        if img is None or img.size == 0:
            return None
        yellow_count, red_count, white_count, black_count, grey_count, wall_count, blue_count, green_count = self.count_image_color_pixels(img)
        if white_count != 0 and black_count != 0:
            return 'S'
        else: return None
    
    def analyze_image(self, converted_img, last_position, last_direction):
        """Image analysis logic, it returns a letter if the image is a sign, otherwise it returns None."""
        if not self.debugging:
            if last_position.distance_to(self.last_token_position) < 0.015 and self.last_token_direction.dot(last_direction) > 0.9:
                return None
        if converted_img is None or converted_img.size == 0:
            return None
        # self.debug_show(converted_img, "analyzing_img")
        result = self.clean_image_with_own_thresh(converted_img)
        contour, _ = result
        if contour is not None:
            yellow_count, red_count, white_count, black_count, grey_count, blue_count, green_count = self.count_color_pixels(converted_img, contour)
            if yellow_count == 0 and red_count == 0 and black_count == 0 and white_count == 0 and grey_count == 0 and blue_count == 0 and green_count == 0: return None            
            if self.is_victim(converted_img):
                victim = self.return_victim_letter(converted_img, contour)
                if victim is not None:
                    self.last_token_position = last_position
                    self.last_token_direction = last_direction
                    return victim
            elif self.is_cognitive(converted_img):
                # hazard = self.return_hazard_letter(yellow_count, red_count, black_count, white_count)
                cognitive = self.return_cognitive_letter(converted_img)
                if cognitive is not None:
                    self.last_token_position = last_position
                    self.last_token_direction = last_direction
                    return cognitive
        else:
            return 'Z'

# image = ImageProcessor()
# # # # "C:\Willow\imgs\ZoomedIn\CD9449.png"
# # # image_test\other test\A4 R3.png
# img = cv2.imread("image_testing/obstacles/ci5318.png")
# # # # "C:\Willow\image_test\post_test4\special front0766.png"
# img = np.dstack((img, np.full([40,40], 255))).astype(np.uint8)

# is_image_zoomed = image.is_image_zoomed_in(img)
# print(f"Is image zoomed in: {is_image_zoomed}")

# # is_cognitive = image.is_cognitive(img)
# # print(f"Is cognitive: {is_cognitive}")

# print(f"Letter: {image.analyze_image(img, Vector(0,0), Vector(1,0))}")

# print(f"Is image: {image.is_image(img)}")
# # grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# # _, thresh = cv2.threshold(grey, 155, 255, cv2.THRESH_BINARY)
# # image.debug_show(thresh, "img")
# # # img_aux = img.copy()
# # # img_aux = cv2.cvtColor(img_aux, cv2.COLOR_BGR2HSV)
# # # image.debug_show(img_aux, "img_aux")
# # # thresh = image.threshold_image(img)
# # # image.debug_show(thresh, "thresh")
# # # image.clean_image_with_own_thresh(img)
# # # print(image.is_image(img))
# # print(image.analyze_image(img, Vector(0,0), Vector(1,0)))
# # # yellow_count, red_count, white_count, black_count, grey_count, wall_count = image.count_image_color_pixels(img)
# # # print(f"white: {white_count}, black: {black_count}, wall: {wall_count}, red: {red_count}, yellow: {yellow_count}, grey: {grey_count}")
# # # is_image_zoomed = image.is_image_zoomed_in(img)
# # # print(f"Is image zoomed in: {is_image_zoomed}")
# # # if is_image_zoomed:
# # #     a = image.analyze_zoomed_image((img), Vector(0,0), Vector(1,0))
# # #     print(f"Analyzed zoomed image: {a}")
# # # contour_centre = image.get_contour_centre(img)
# # # print(f"Contour centre: {contour_centre}")
# # # print(image.analyze_zoomed_image(img, Vector(0,0), Vector(1,0)))
# # # print(image.analyze_image(img, Vector(0,0), Vector(1,0)))
# # # contour_centre = image.get_contour_centre(img)
# # # print(f"Contour centre: {contour_centre}")
# print("---------------------------------------------------------------------------------------------------")