"""Configure the Boson camera
Harshith Mohan Kumar
1/19/25
"""

def configure_boson(boson):
    # Set Boson to external sync slave mode
    boson.set_external_sync_mode(2)  # 2 is for slave mode
    # boson.set_cmos_output_mode(mode="one-shot")