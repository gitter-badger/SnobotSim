package com.snobot.simulator.gui.module_widget;

import java.text.DecimalFormat;
import java.util.Collection;
import java.util.Map.Entry;

import javax.swing.JDialog;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.border.TitledBorder;

import com.snobot.simulator.gui.module_widget.settings.EncoderSettingsDialog;
import com.snobot.simulator.gui.module_widget.settings.SimpleSettingsDialog;
import com.snobot.simulator.jni.module_wrapper.DigitalSourceWrapperJni;
import com.snobot.simulator.jni.module_wrapper.EncoderWrapperJni;
import com.snobot.simulator.jni.module_wrapper.GyroWrapperJni;

public class GyroGraphicDisplay extends BaseWidgetDisplay<Integer, GyroWrapperDisplay>
{

    public GyroGraphicDisplay(Collection<Integer> aKeys)
    {
        super(aKeys);
        setBorder(new TitledBorder("Gyros"));
    }

    @Override
    public void update()
    {
        for (Entry<Integer, GyroWrapperDisplay> pair : mWidgetMap.entrySet())
        {
            int key = pair.getKey();
            double angle = GyroWrapperJni.getAngle(key);

            pair.getValue().updateDisplay(angle);
        }
    }

    @Override
    protected GyroWrapperDisplay createWidget(Integer pair)
    {
        return new GyroWrapperDisplay();
    }

    @Override
    protected JDialog createSettingsDialog(Integer aKey)
    {
        SimpleSettingsDialog dialog = new SimpleSettingsDialog("Gyro " + aKey + " Settings", aKey, getName(aKey))
        {
            @Override
            protected void onSubmit()
            {
                DigitalSourceWrapperJni.setName(aKey, getComponentName());
                mLabelMap.get(aKey).setText(getComponentName());
            }
        };

        dialog.pack();

        return dialog;
    }

    @Override
    protected String getName(Integer aKey)
    {
        return GyroWrapperJni.getName(aKey);
    }
}

class GyroWrapperDisplay extends JPanel
{

    private JTextField mAngleField;

    public GyroWrapperDisplay()
    {
        mAngleField = new JTextField(6);
        add(mAngleField);
    }

    public void updateDisplay(double aAngle)
    {
        mAngleField.setText("" + aAngle);
    }
}