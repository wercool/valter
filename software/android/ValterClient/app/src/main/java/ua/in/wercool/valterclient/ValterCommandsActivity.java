package ua.in.wercool.valterclient;

import android.content.SharedPreferences;
import android.support.design.widget.FloatingActionButton;
import android.support.design.widget.Snackbar;
import android.support.design.widget.TabLayout;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;

import android.support.v4.app.Fragment;
import android.support.v4.app.FragmentManager;
import android.support.v4.app.FragmentPagerAdapter;
import android.support.v4.view.ViewPager;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;

import android.widget.TextView;

import valter.Valter;

public class ValterCommandsActivity extends AppCompatActivity {

    private SectionsPagerAdapter mSectionsPagerAdapter;

    public static String PREFS_NAME = "ValterCommandsActivity";

    public static boolean dialogMode = false;

    private ViewPager mViewPager;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        if (ValterCommandsActivity.dialogMode) {
            setTheme(R.style.AppTheme_Dialog);
        }

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_valter_commands);

        // Create the adapter that will return a fragment for each of the three
        // primary sections of the activity.
        mSectionsPagerAdapter = new SectionsPagerAdapter(getSupportFragmentManager());

        // Set up the ViewPager with the sections adapter.
        mViewPager = (ViewPager) findViewById(R.id.container);
        mViewPager.setAdapter(mSectionsPagerAdapter);
        mViewPager.addOnPageChangeListener(new ViewPager.OnPageChangeListener() {
            @Override
            public void onPageScrolled(int position, float positionOffset, int positionOffsetPixels) {

            }

            @Override
            public void onPageSelected(int position) {
                SharedPreferences settings = getSharedPreferences(ValterCommandsActivity.PREFS_NAME, 0);
                SharedPreferences.Editor editor = settings.edit();
                editor.putInt("lastSelectedCommandTab", position);
                editor.commit();
            }

            @Override
            public void onPageScrollStateChanged(int state) {

            }
        });

        if (ValterCommandsActivity.dialogMode) {
            getWindow().setLayout(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT);
        }

        SharedPreferences settings = getSharedPreferences(ValterCommandsActivity.PREFS_NAME, 0);
        int lastSelectedCommandTab = settings.getInt("lastSelectedCommandTab", 0);
        Log.i("ValterCommands", String.format("Last Selected Tab: %d", lastSelectedCommandTab));

        mViewPager.setCurrentItem(lastSelectedCommandTab);
    }


    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_valter_commands, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }


    public class SectionsPagerAdapter extends FragmentPagerAdapter {

        public SectionsPagerAdapter(FragmentManager fm) {
            super(fm);
        }


        @Override
        public Fragment getItem(int position) {
            // getItem is called to instantiate the fragment for the given page.
            // Return a PlaceholderFragment (defined as a static inner class below).
            Log.i("Valter Commands", String.format("Selected Tab %d", position));

            Fragment fragment = null;

            switch (position) {
                case 0:
                    fragment = new PlatformControlP1Commands();
                break;
                case 1:
                    fragment = new PlatformControlP2Commands();
                break;
                case 2:
                    fragment = new PlatformLocationp1Commands();
                break;
                case 3:
                    fragment = new PlatformManipulatorAndIRBumperCommands();
                break;
                case 4:
                    fragment = new BodyControlP1Commands();
                break;
                case 5:
                    fragment = new ArmControlRightCommands();
                break;
                case 6:
                    fragment = new ArmControlLeftCommands();
                break;
            }

            return fragment;
        }

        @Override
        public int getCount() {
            return 7;
        }

        @Override
        public CharSequence getPageTitle(int position) {
            switch (position) {
                case 0:
                    return "PLATFORM-CONTROL-P1";
                case 1:
                    return "PLATFORM-CONTROL-P2";
                case 2:
                    return "PLATFORM-LOCATION-P1";
                case 3:
                    return "PLATFORM-MANIPULATOR-AND-IR-BUMPER";
                case 4:
                    return "BODY-CONTROL-P1";
                case 5:
                    return "ARM-CONTROL-RIGHT";
                case 6:
                    return "ARM-CONTROL-LEFT";
            }
            return null;
        }
    }
}
