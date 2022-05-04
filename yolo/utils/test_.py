from utils.models import Model


model = Model("C:\\Users\\c\\Downloads\\last.pt")
color_image = "C:\\Users\\c\\test_imaage_for_vex.png"
pred = model.predict(color_image, color_image.shape, conf_thres=0.4)