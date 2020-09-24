#Simple assignment
from selenium.webdriver import Firefox
from selenium.webdriver.common.by import By
from selenium.webdriver.common.action_chains import ActionChains
from selenium.webdriver.common.keys import Keys
import random

def init_driver():
    driver = Firefox()
    # Access each dimension individually
    driver.maximize_window()
    return driver

def check_window_size(driver, min_width, min_height):    
    width = driver.get_window_size().get("width")
    height = driver.get_window_size().get("height")
    if width < min_width:
        print("Window width(" + str(width) + ") is lower than required for the test(" + str(min_width))
        return False
    if height < min_height:
        print("Window height(" + str(height) + ") is lower than required for the test(" + str(min_height))
        return False
    return True

def connect_with_server(driver, address):
    expected_title = "Route Admin Panel"
    driver.get(address)
    if driver.current_url != address:
        print("Current url: " + driver.current_url)
        print("Can not connect to: " + address)
        return False
    if driver.title != expected_title:
        print("Window title is '" + driver.title + "' while should be '" + expected_title + "'")
        return False
    return True

def add_point_save_current_pos(driver, point_label):
    driver.find_element(By.ID, "save-pos").click()
    save_current_pos_dialog = driver.find_element(By.ID, "saveCurrentPosDialog")
    dialog_visible = save_current_pos_dialog.is_displayed()
    if dialog_visible == True:
        driver.find_element(By.ID, "currentPosLabel").clear()
        driver.find_element(By.ID, "currentPosLabel").send_keys(point_label)
        driver.find_element(By.ID, "saveCurrentPosConfirm").click()
    else:
        print("Dialog did not open. Test failed")
        return False
    return True

def add_points_save_current_pos(driver, point_label, count):
    for i in range(count):
        assert add_point_save_current_pos(driver, point_label + " " + str(i))
    return True

def delete_point(driver, table_row):
    row_id = str(table_row.get_property("id"))
    if len(row_id) == 0:
        print("Row ID is empty")
        return False
    row_cells = table_row.find_elements(By.TAG_NAME, "td")
    if len(row_cells)!=5:
        print("Table row contain " + str(len(row_cells)) + " instead of 5")
    label = str(row_cells[3].text)
    go_button = row_cells[4].find_elements(By.TAG_NAME, "button")[0].find_element(By.TAG_NAME, "i")
    go_button_class = go_button.get_attribute("class")
    if go_button_class != 'fa fa-angle-double-right':
        print("Can not find go button")
        return False
    del_button = row_cells[4].find_elements(By.TAG_NAME, "button")[1].find_element(By.TAG_NAME, "i")
    del_button_class = del_button.get_attribute("class")
    if del_button_class == 'fa fa-trash-alt':
        print("Delete point with label " + label + " at row #ID " + str(row_id))
        # Cannot be done with simple version as below
        # del_button.click()
        # When table has more than 6 rows, some of the elements are hidden and
        # return error when trying to click them
        driver.execute_script("arguments[0].focus();", del_button)
        driver.execute_script("arguments[0].click();", del_button)
    else:
        print("Can not find delete button")
        return False
    return True

def delete_points(driver, rows, order):
    if order == "ascending":
        for row in rows:
            try:
                assert delete_point(driver, row)
            except AssertionError:
                print("Assertion failed: Delete point ascending")
                return False
    elif order == "descending":
        for row in reversed(rows): 
            try:
                assert delete_point(driver, row)
            except AssertionError:
                print("Assertion failed: Delete point descending")
                return False
    elif order == "random":
        for i in range(len(rows)):
            try:
                rand_val = random.randint(0, len(rows)-1)
                row = rows.pop(rand_val)
                assert delete_point(driver, row)
            except AssertionError:
                print("Assertion failed: Delete point descending")
                return False
    return True

def add_points(driver, count):
    try:
        assert add_points_save_current_pos(driver, 'point', count)
    except AssertionError:
        print("Assertion failed: Add points")

def verify_points_count(driver, count):
    targets_table = driver.find_element(By.ID, "targets")
    targets_table_body = targets_table.find_element(By.TAG_NAME, "tbody")
    point_rows = targets_table_body.find_elements(By.TAG_NAME, "tr")
    try:
        assert len(point_rows) == count
    except AssertionError:
        print("Assertion failed: Check number of items in table")
    return point_rows


def main():
    print("Init browser")
    driver = init_driver()
    point_count = 5

    try:
        assert check_window_size(driver, 1200, 900)
    except AssertionError:
        print("Assertion failed: check window size")

    try:
        assert connect_with_server(driver, "http://localhost:3000/")
    except AssertionError:
        print("Assertion failed: Connect with server")
    
    print("Test #1, 5 ascending points")
    add_points(driver, point_count)
    point_rows = verify_points_count(driver, point_count)
    delete_points(driver, point_rows, 'ascending')

    print("Test #2, 5 descending points")
    add_points(driver, point_count)
    point_rows = verify_points_count(driver, point_count)
    delete_points(driver, point_rows, 'descending')

    try:
        for i in range(1, 25):
            print("Test #" + str(i+2) + ", " + str(i) + " random points")
            add_points(driver, i)
            point_rows = verify_points_count(driver, i)
            delete_points(driver, point_rows, 'random')
    except Exception as error:
        print(error)
        print("Test exception occured")
    driver.close()
    driver.quit()

if __name__ == "__main__":
    main()
